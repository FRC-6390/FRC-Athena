import { describe, expect, test } from 'bun:test';
import { connectionTargets } from './connection-targets';
import {
  DEFAULT_BRIDGE_URL,
  probeBridge,
  resolveSource,
  sourceFromQuery,
  validBridgeUrl,
  type ProbeSocket
} from './source-select';

/** A probe socket that opens, fails or never answers, on demand. */
function fakeProbe(behaviour: 'open' | 'error' | 'silent') {
  return (): ProbeSocket => {
    const socket: ProbeSocket = { onopen: null, onerror: null, onclose: null, close: () => undefined };
    queueMicrotask(() => {
      if (behaviour === 'open') socket.onopen?.({});
      if (behaviour === 'error') socket.onerror?.({});
    });
    return socket;
  };
}

describe('source selection from the query', () => {
  test('the mock wins with ?source=mock, even inside Tauri', () => {
    expect(sourceFromQuery('?source=mock', true)).toEqual({ kind: 'mock' });
    expect(sourceFromQuery('?source=mock', false)).toEqual({ kind: 'mock' });
  });

  test('Tauri always uses the live source', () => {
    expect(sourceFromQuery('', true)).toEqual({ kind: 'live' });
    expect(sourceFromQuery('?bridge=ws://127.0.0.1:5811', true)).toEqual({ kind: 'live' });
  });

  test('a browser with ?bridge= uses that bridge', () => {
    expect(sourceFromQuery('?bridge=ws://127.0.0.1:5811', false)).toEqual({ kind: 'bridge', url: 'ws://127.0.0.1:5811/' });
    expect(sourceFromQuery('?bridge=wss://dash.example:9/x', false)).toEqual({
      kind: 'bridge',
      url: 'wss://dash.example:9/x'
    });
  });

  test('an invalid ?bridge= falls back to the mock', () => {
    expect(sourceFromQuery('?bridge=http://127.0.0.1:5811', false)).toEqual({ kind: 'mock' });
    expect(validBridgeUrl('not a url')).toBeNull();
  });

  test('a plain browser page asks for a probe of the default bridge', () => {
    expect(sourceFromQuery('', false)).toBe('probe');
    expect(sourceFromQuery('?ui=classic', false)).toBe('probe');
  });

  test('resolveSource uses the default bridge when it answers, the mock otherwise', async () => {
    const probed: string[] = [];
    const answers = async (url: string) => {
      probed.push(url);
      return true;
    };
    expect(await resolveSource('', false, answers)).toEqual({ kind: 'bridge', url: DEFAULT_BRIDGE_URL });
    expect(probed).toEqual([DEFAULT_BRIDGE_URL]);
    expect(await resolveSource('', false, async () => false)).toEqual({ kind: 'mock' });
  });

  test('resolveSource does not probe when the query decides', async () => {
    const mustNotProbe = async (): Promise<boolean> => {
      throw new Error('probed');
    };
    expect(await resolveSource('?source=mock', false, mustNotProbe)).toEqual({ kind: 'mock' });
    expect(await resolveSource('', true, mustNotProbe)).toEqual({ kind: 'live' });
    expect(await resolveSource('?bridge=ws://127.0.0.1:5811', false, mustNotProbe)).toEqual({
      kind: 'bridge',
      url: 'ws://127.0.0.1:5811/'
    });
  });
});

describe('bridge probe', () => {
  test('true when the socket opens', async () => {
    expect(await probeBridge(DEFAULT_BRIDGE_URL, 200, fakeProbe('open'))).toBe(true);
  });

  test('false on an error or a socket that never opens', async () => {
    expect(await probeBridge(DEFAULT_BRIDGE_URL, 200, fakeProbe('error'))).toBe(false);
    expect(await probeBridge(DEFAULT_BRIDGE_URL, 10, fakeProbe('silent'))).toBe(false);
  });

  test('false when the socket cannot be created', async () => {
    expect(
      await probeBridge(DEFAULT_BRIDGE_URL, 200, () => {
        throw new Error('no WebSocket');
      })
    ).toBe(false);
  });
});

describe('quick-connect targets', () => {
  test('with a team number the radio and mDNS hosts are filled in', () => {
    const targets = connectionTargets(6390);
    expect(targets).toEqual([
      { label: 'Simulator', host: '127.0.0.1', needsTeam: false },
      { label: 'Robot (USB)', host: '172.22.11.2', needsTeam: false },
      { label: 'Robot (radio)', host: '10.63.90.2', needsTeam: false },
      { label: 'Robot (mDNS)', host: 'roborio-6390-frc.local', needsTeam: false }
    ]);
  });

  test('without a team the placeholders stay and need a team', () => {
    const targets = connectionTargets();
    expect(targets.map((t) => t.host)).toEqual([
      '127.0.0.1',
      '172.22.11.2',
      '10.TE.AM.2',
      'roborio-TEAM-frc.local'
    ]);
    expect(targets.map((t) => t.needsTeam)).toEqual([false, false, true, true]);
  });

  test('an invalid team number keeps the placeholders', () => {
    expect(connectionTargets(0)[2]).toEqual({ label: 'Robot (radio)', host: '10.TE.AM.2', needsTeam: true });
    expect(connectionTargets(10000)[3].needsTeam).toBe(true);
  });
});
