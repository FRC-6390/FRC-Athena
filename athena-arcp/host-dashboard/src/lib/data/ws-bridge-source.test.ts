import { describe, expect, test } from 'bun:test';
import { encodeDictionary, encodeFrame } from './encoder';
import { fromBridgeValue, toBridgeValue, WsBridgeSource, type BridgeSocket } from './ws-bridge-source';
import type { DecodedRecord } from './types';
import { TYPE_TAG } from './types';

const OPEN = 1;
const CLOSED = 3;

/** A WebSocket stand-in that the test drives: open it, receive messages, drop it. */
class FakeSocket implements BridgeSocket {
  binaryType = '';
  readyState = 0;
  sent: string[] = [];
  onopen: ((event?: unknown) => void) | null = null;
  onmessage: ((event: { data: unknown }) => void) | null = null;
  onclose: ((event?: unknown) => void) | null = null;
  onerror: ((event?: unknown) => void) | null = null;

  send(data: string): void {
    if (this.readyState !== OPEN) throw new Error('socket is not open');
    this.sent.push(data);
  }

  close(): void {
    if (this.readyState === CLOSED) return;
    this.readyState = CLOSED;
    this.onclose?.({});
  }

  open(): void {
    this.readyState = OPEN;
    this.onopen?.({});
  }

  receive(data: unknown): void {
    this.onmessage?.({ data });
  }

  drop(): void {
    this.readyState = CLOSED;
    this.onclose?.({});
  }

  lastCommand(): Record<string, unknown> {
    return JSON.parse(this.sent[this.sent.length - 1]);
  }
}

function harness(options: { requestTimeoutMs?: number; reconnectMs?: number } = {}) {
  const sockets: FakeSocket[] = [];
  const source = new WsBridgeSource({
    url: 'ws://127.0.0.1:5811/',
    createSocket: () => {
      const socket = new FakeSocket();
      sockets.push(socket);
      return socket;
    },
    requestTimeoutMs: options.requestTimeoutMs ?? 1_000,
    reconnectMs: options.reconnectMs ?? 10_000
  });
  const batches: DecodedRecord[][] = [];
  source.start({ records: (records) => batches.push(records) });
  return { source, sockets, batches, socket: () => sockets[sockets.length - 1] };
}

const tick = () => new Promise((resolve) => setTimeout(resolve, 5));

describe('WsBridgeSource commands', () => {
  test('write sends a JSON command with an id and resolves with the robot reply', async () => {
    const { source, socket } = harness();
    socket().open();
    const pending = source.commands.write(7, 3, 2.5);
    expect(socket().lastCommand()).toEqual({ id: 1, cmd: 'write', signalId: 3, value: 2.5 });
    socket().receive(
      JSON.stringify({ id: 1, ok: true, result: { accepted: false, reason: 'rejected: locked (InMatch)', actualValue: 0 } })
    );
    const reply = await pending;
    expect(reply).toMatchObject({
      requestId: 7,
      op: 'write',
      signalId: 3,
      accepted: false,
      reason: 'rejected: locked (InMatch)',
      actualValue: 0
    });
  });

  test('run and cancel send action commands', async () => {
    const { source, socket } = harness();
    socket().open();
    const run = source.commands.run(1, 9);
    expect(socket().lastCommand()).toEqual({ id: 1, cmd: 'action', signalId: 9, run: true });
    socket().receive(JSON.stringify({ id: 1, ok: true, result: { accepted: true, reason: '' } }));
    expect((await run).accepted).toBe(true);

    const cancel = source.commands.cancel(2, 9);
    expect(socket().lastCommand()).toEqual({ id: 2, cmd: 'action', signalId: 9, run: false });
    socket().receive(JSON.stringify({ id: 2, ok: true, result: { accepted: false, reason: 'no such action' } }));
    expect(await cancel).toMatchObject({ op: 'cancel', accepted: false, reason: 'no such action' });
  });

  test('error replies reject the request with the bridge message', async () => {
    const { source, socket } = harness();
    socket().open();
    const pending = source.commands.write(1, 99, 1);
    socket().receive(JSON.stringify({ id: 1, ok: false, error: 'unknown signal 99' }));
    await expect(pending).rejects.toThrow('unknown signal 99');
  });

  test('requests fail while the bridge is not open', async () => {
    const { source } = harness();
    await expect(source.commands.run(1, 2)).rejects.toThrow('not connected to the bridge');
  });

  test('an unanswered request times out', async () => {
    const { source, socket } = harness({ requestTimeoutMs: 15 });
    socket().open();
    await expect(source.commands.run(1, 2)).rejects.toThrow('did not answer action');
  });

  test('connect sends host and port and resolves the session info', async () => {
    const { source, socket } = harness();
    socket().open();
    const pending = source.connect({ host: '127.0.0.1', controlPort: 5812 });
    expect(socket().lastCommand()).toEqual({ id: 1, cmd: 'connect', host: '127.0.0.1', port: 5812, replay: false });
    socket().receive(
      JSON.stringify({ id: 1, ok: true, result: { sessionId: 2, udpPort: 40000, controlAddr: '127.0.0.1:5812' } })
    );
    expect(await pending).toEqual({ sessionId: 2, udpPort: 40000, controlAddr: '127.0.0.1:5812' });
  });

  test('pending requests fail when the socket drops', async () => {
    const { source, socket } = harness();
    socket().open();
    const pending = source.commands.run(1, 2);
    socket().drop();
    await expect(pending).rejects.toThrow('bridge connection closed');
  });
});

describe('WsBridgeSource messages', () => {
  test('binary messages decode into the sink in arrival order', async () => {
    const { socket, batches } = harness();
    socket().open();
    socket().receive(new Uint8Array(encodeDictionary([{ signalId: 1, path: 'robot/a', typeTag: TYPE_TAG.f64 }])).buffer);
    socket().receive(
      encodeFrame({
        keyframe: true,
        cycleTimestampMicros: 1_000,
        sequence: 1,
        entries: [{ signalId: 1, typeTag: TYPE_TAG.f64, value: 2.5 }]
      }).buffer
    );
    await tick();
    expect(batches.map((b) => b.map((r) => r.kind))).toEqual([['dictionary'], ['frame']]);
    const dictionary = batches[0][0];
    expect(dictionary.kind === 'dictionary' && dictionary.entries[0].path).toBe('robot/a');
  });

  test('a reconnect starts with a fresh decoder and binary input is arraybuffer', async () => {
    const { source, sockets, socket, batches } = harness({ reconnectMs: 1 });
    socket().open();
    socket().drop();
    await tick();
    expect(sockets.length).toBe(2);
    expect(socket().binaryType).toBe('arraybuffer');
    socket().open();
    expect(source.isOpen).toBe(true);
    socket().receive(new Uint8Array(encodeDictionary([{ signalId: 4, path: 'robot/b', typeTag: TYPE_TAG.i64 }])).buffer);
    await tick();
    expect(batches.length).toBe(1);
  });

  test('status and gap events reach their listeners', () => {
    const { source, socket } = harness();
    const statuses: unknown[] = [];
    const events: unknown[] = [];
    source.onStatus((status) => statuses.push(status));
    source.onEvent((event) => events.push(event));
    socket().open();
    socket().receive(JSON.stringify({ event: 'status', status: { connected: true, frames: 12 } }));
    socket().receive(JSON.stringify({ event: 'gap', from: 4, to: 6, lost: 3 }));
    socket().receive(JSON.stringify({ event: 'disconnected', reason: 'robot lost' }));
    expect(statuses).toEqual([{ connected: true, frames: 12 }]);
    expect(events).toEqual([
      { kind: 'gap', from: 4, to: 6, lost: 3 },
      { kind: 'disconnected', reason: 'robot lost' }
    ]);
  });

  test('non-JSON text is ignored without a crash', () => {
    const { socket } = harness();
    socket().open();
    expect(() => socket().receive('not json')).not.toThrow();
  });

  test('stop closes the socket and does not reconnect', async () => {
    const { source, sockets } = harness({ reconnectMs: 1 });
    sockets[0].open();
    source.stop();
    await tick();
    expect(sockets.length).toBe(1);
    expect(source.isOpen).toBe(false);
  });
});

describe('value conversion for the bridge', () => {
  test('bigints and integer arrays become numbers', () => {
    expect(toBridgeValue(5n)).toBe(5);
    expect(toBridgeValue([1n, 2])).toEqual([1, 2]);
    expect(toBridgeValue(true)).toBe(true);
  });

  test('struct values cannot be written through the bridge', () => {
    expect(() => toBridgeValue({ schemaId: 1, bytes: new Uint8Array([1]) })).toThrow('struct values');
  });

  test('replies convert structs from base64 and null stays null', () => {
    expect(fromBridgeValue({ schemaId: 2, base64: 'AQI=' })).toEqual({ schemaId: 2, bytes: new Uint8Array([1, 2]) });
    expect(fromBridgeValue(null)).toBeNull();
    expect(fromBridgeValue('x')).toBe('x');
  });
});
