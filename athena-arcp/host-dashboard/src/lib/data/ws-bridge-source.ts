/**
 * Browser data source: the robot (or simulator) through the `arcp-ws-bridge` WebSocket (see
 * `athena-arcp/rust/crates/arcp-client/src/ws_bridge.rs` for the protocol). Records arrive as E1-W bytes
 * in binary messages and go through the same decoder and store as the live and mock sources. Commands are
 * JSON requests answered by the bridge with the robot's reply (E5). The socket reconnects on its own; a
 * reconnect starts a fresh decode context, and the bridge resends the dictionary.
 */
import { createDecodeContext, decodeRecords, type DecodeContext } from './decoder';
import type { CommandReply, RobotCommands } from './commands';
import type { DataSource, SourceSink } from './source';
import type { LiveEvent, LiveStatus } from './live-source';
import type { SignalValue, StructValue } from './types';
import { decodeBase64 } from './base64';
import { ARCP_CONTROL_PORT } from './arcp-ports';

const OPEN = 1;
const DEFAULT_REQUEST_TIMEOUT_MS = 5_000;
const DEFAULT_RECONNECT_MS = 1_000;

/** The part of the WebSocket API the source uses. Tests pass a fake. */
export interface BridgeSocket {
  binaryType: string;
  readonly readyState: number;
  onopen: ((event?: unknown) => void) | null;
  onmessage: ((event: { data: unknown }) => void) | null;
  onclose: ((event?: unknown) => void) | null;
  onerror: ((event?: unknown) => void) | null;
  send(data: string): void;
  close(): void;
}

export type WsBridgeOptions = {
  url: string;
  createSocket?: (url: string) => BridgeSocket;
  reconnectMs?: number;
  requestTimeoutMs?: number;
};

export type BridgeConnectOptions = { host: string; controlPort?: number; replay?: boolean };
export type BridgeConnectInfo = { sessionId: number; udpPort: number; udpFallback?: boolean; controlAddr: string };

type Pending = { resolve: (value: unknown) => void; reject: (error: Error) => void; timer: ReturnType<typeof setTimeout> };

type JsonValue = unknown;

/** Signal value to the bridge's JSON. The bridge converts numbers to the signal's type. */
export function toBridgeValue(value: SignalValue): JsonValue {
  if (typeof value === 'bigint') return Number(value);
  if (Array.isArray(value)) return value.map((item) => (typeof item === 'bigint' ? Number(item) : item));
  if (typeof value === 'object' && value !== null) {
    throw new Error('struct values cannot be written through the browser bridge');
  }
  return value;
}

/** The bridge's JSON reply value back to a signal value. Structs arrive as `{schemaId, base64}`. */
export function fromBridgeValue(json: JsonValue): SignalValue | null {
  if (json === null || json === undefined) return null;
  if (typeof json === 'object' && !Array.isArray(json)) {
    const s = json as { schemaId: number; base64: string };
    const value: StructValue = { schemaId: s.schemaId, bytes: decodeBase64(s.base64) };
    return value;
  }
  return json as SignalValue;
}

export class WsBridgeSource implements DataSource {
  readonly name = 'bridge';
  readonly commands: RobotCommands;
  readonly url: string;
  private readonly createSocket: (url: string) => BridgeSocket;
  private readonly reconnectMs: number;
  private readonly requestTimeoutMs: number;
  private sink: SourceSink | null = null;
  private socket: BridgeSocket | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private stopped = true;
  private ctx: DecodeContext = createDecodeContext();
  private chain: Promise<void> = Promise.resolve();
  private nextId = 1;
  private readonly pending = new Map<number, Pending>();
  private readonly statusListeners = new Set<(status: LiveStatus) => void>();
  private readonly eventListeners = new Set<(event: LiveEvent) => void>();
  private readonly openListeners = new Set<(open: boolean) => void>();

  constructor(options: WsBridgeOptions) {
    this.url = options.url;
    this.createSocket = options.createSocket ?? ((url) => new WebSocket(url) as unknown as BridgeSocket);
    this.reconnectMs = options.reconnectMs ?? DEFAULT_RECONNECT_MS;
    this.requestTimeoutMs = options.requestTimeoutMs ?? DEFAULT_REQUEST_TIMEOUT_MS;
    this.commands = {
      run: (requestId, signalId) => this.action(requestId, signalId, true),
      cancel: (requestId, signalId) => this.action(requestId, signalId, false),
      write: async (requestId, signalId, value) => {
        const reply = (await this.request('write', { signalId, value: toBridgeValue(value) })) as {
          accepted: boolean;
          reason: string;
          actualValue: JsonValue;
        };
        return {
          requestId,
          op: 'write',
          signalId,
          accepted: reply.accepted,
          reason: reply.reason,
          actualValue: fromBridgeValue(reply.actualValue),
          receivedAtMs: Date.now()
        } satisfies CommandReply;
      }
    };
  }

  /** True while the socket to the bridge is open (the robot may still be disconnected). */
  get isOpen(): boolean {
    return this.socket !== null && this.socket.readyState === OPEN;
  }

  start(sink: SourceSink): void {
    this.sink = sink;
    this.stopped = false;
    this.open();
  }

  stop(): void {
    this.stopped = true;
    if (this.reconnectTimer !== null) clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
    const socket = this.socket;
    this.socket = null;
    socket?.close();
    this.failPending(new Error('bridge stopped'));
    this.sink = null;
  }

  onStatus(listener: (status: LiveStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }

  onEvent(listener: (event: LiveEvent) => void): () => void {
    this.eventListeners.add(listener);
    return () => this.eventListeners.delete(listener);
  }

  /** Called when the bridge socket opens or closes. */
  onOpenChange(listener: (open: boolean) => void): () => void {
    this.openListeners.add(listener);
    return () => this.openListeners.delete(listener);
  }

  connect(options: BridgeConnectOptions): Promise<BridgeConnectInfo> {
    return this.request('connect', {
      host: options.host,
      port: options.controlPort ?? ARCP_CONTROL_PORT,
      replay: options.replay ?? false
    }) as Promise<BridgeConnectInfo>;
  }

  disconnect(): Promise<void> {
    return this.request('disconnect').then(() => undefined);
  }

  status(): Promise<LiveStatus> {
    return this.request('status') as Promise<LiveStatus>;
  }

  subscribe(pattern: string, hz?: number): Promise<number> {
    return this.request('subscribe', { pattern, hz: hz ?? null }) as Promise<number>;
  }

  timeSync(samples = 8): Promise<{ offsetMicros: number; rttMicros: number; samples: number }> {
    return this.request('timeSync', { samples }) as Promise<{ offsetMicros: number; rttMicros: number; samples: number }>;
  }

  fileList(): Promise<{ name: string; size: number; modifiedUnixSecs: number }[]> {
    return this.request('fileList') as Promise<{ name: string; size: number; modifiedUnixSecs: number }[]>;
  }

  async fileDownload(name: string): Promise<Uint8Array> {
    return decodeBase64((await this.request('fileDownload', { name })) as string);
  }

  fileDelete(name: string): Promise<void> {
    return this.request('fileDelete', { name }).then(() => undefined);
  }

  private async action(requestId: number, signalId: number, run: boolean): Promise<CommandReply> {
    const reply = (await this.request('action', { signalId, run })) as { accepted: boolean; reason: string };
    return {
      requestId,
      op: run ? 'run' : 'cancel',
      signalId,
      accepted: reply.accepted,
      reason: reply.reason,
      actualValue: null,
      receivedAtMs: Date.now()
    };
  }

  /** Sends one command and resolves with its `result`. Rejects on `ok: false`, a closed socket or a timeout. */
  request(cmd: string, body: Record<string, JsonValue> = {}): Promise<JsonValue> {
    const socket = this.socket;
    if (!socket || socket.readyState !== OPEN) {
      return Promise.reject(new Error('not connected to the bridge'));
    }
    const id = this.nextId++;
    return new Promise<JsonValue>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`bridge did not answer ${cmd} within ${this.requestTimeoutMs} ms`));
      }, this.requestTimeoutMs);
      this.pending.set(id, { resolve, reject, timer });
      try {
        socket.send(JSON.stringify({ id, cmd, ...body }));
      } catch (error) {
        clearTimeout(timer);
        this.pending.delete(id);
        reject(error instanceof Error ? error : new Error(String(error)));
      }
    });
  }

  /** Feeds one text message (reply or event) to the waiting request or listeners. */
  handleText(text: string): void {
    let message: { id?: number; ok?: boolean; result?: JsonValue; error?: string; event?: string; [key: string]: unknown };
    try {
      message = JSON.parse(text);
    } catch {
      console.error('bridge sent non-JSON text');
      return;
    }
    if (message.event !== undefined) {
      this.dispatchEvent(message);
      return;
    }
    const waiting = message.id === undefined ? undefined : this.pending.get(message.id);
    if (!waiting || message.id === undefined) return;
    clearTimeout(waiting.timer);
    this.pending.delete(message.id);
    if (message.ok) waiting.resolve(message.result);
    else waiting.reject(new Error(message.error ?? 'bridge error'));
  }

  /** Feeds one binary message (E1-W records) to the decoder. Batches are decoded in arrival order. */
  handleBinary(bytes: Uint8Array): void {
    this.chain = this.chain
      .then(async () => {
        const records = await decodeRecords(bytes, this.ctx);
        this.sink?.records(records);
      })
      .catch((error: unknown) => console.error('bridge batch dropped', error));
  }

  private dispatchEvent(message: { event?: string; [key: string]: unknown }): void {
    if (message.event === 'status') {
      const status = message.status as LiveStatus;
      this.statusListeners.forEach((listener) => listener(status));
      return;
    }
    const { event, ...rest } = message;
    if (event === 'gap' || event === 'backfill' || event === 'disconnected') {
      const liveEvent = { kind: event, ...rest } as unknown as LiveEvent;
      this.eventListeners.forEach((listener) => listener(liveEvent));
    }
  }

  private open(): void {
    if (this.stopped) return;
    const socket = this.createSocket(this.url);
    socket.binaryType = 'arraybuffer';
    this.socket = socket;
    socket.onopen = () => {
      // A new connection gets a new dictionary from the bridge, so decoding starts clean.
      this.ctx = createDecodeContext();
      this.openListeners.forEach((listener) => listener(true));
    };
    socket.onmessage = (event) => {
      if (typeof event.data === 'string') this.handleText(event.data);
      else if (event.data instanceof ArrayBuffer) this.handleBinary(new Uint8Array(event.data));
    };
    const closed = () => {
      if (this.socket !== socket) return;
      this.socket = null;
      this.failPending(new Error('bridge connection closed'));
      this.openListeners.forEach((listener) => listener(false));
      if (!this.stopped) {
        this.reconnectTimer = setTimeout(() => {
          this.reconnectTimer = null;
          this.open();
        }, this.reconnectMs);
      }
    };
    socket.onclose = closed;
    socket.onerror = () => undefined;
  }

  private failPending(error: Error): void {
    for (const [id, waiting] of this.pending) {
      clearTimeout(waiting.timer);
      waiting.reject(error);
      this.pending.delete(id);
    }
  }
}

