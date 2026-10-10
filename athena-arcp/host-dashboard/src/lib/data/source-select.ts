/**
 * Chooses the data source for the dashboard shell. Pure except for the optional reachability probe.
 *
 * Order: `?source=mock` always wins; inside the Tauri webview the live source is used; in a browser
 * `?bridge=ws://host:port` selects the WebSocket bridge (`arcp-ws-bridge`); with no parameter the default
 * bridge is used when it answers, and the mock robot otherwise.
 */

export const DEFAULT_BRIDGE_URL = 'ws://127.0.0.1:5811';
const PROBE_TIMEOUT_MS = 500;

export type SourceChoice = { kind: 'live' } | { kind: 'bridge'; url: string } | { kind: 'mock' };

/** A `ws:` or `wss:` URL, or null. */
export function validBridgeUrl(value: string): string | null {
  try {
    const url = new URL(value);
    return url.protocol === 'ws:' || url.protocol === 'wss:' ? url.toString() : null;
  } catch {
    return null;
  }
}

/** The choice for a query string, or `'probe'` when only a reachability check can decide (browser, no parameter). */
export function sourceFromQuery(search: string, tauri: boolean): SourceChoice | 'probe' {
  const params = new URLSearchParams(search);
  if (params.get('source') === 'mock') return { kind: 'mock' };
  if (tauri) return { kind: 'live' };
  const bridge = params.get('bridge');
  if (bridge !== null) {
    const url = validBridgeUrl(bridge);
    if (url) return { kind: 'bridge', url };
    console.warn(`ignoring invalid ?bridge=${bridge}; using the mock robot`);
    return { kind: 'mock' };
  }
  return 'probe';
}

/** True when a WebSocket to `url` opens within `timeoutMs`. The socket is closed again at once. */
export function probeBridge(
  url: string,
  timeoutMs = PROBE_TIMEOUT_MS,
  createSocket: (url: string) => ProbeSocket = (u) => new WebSocket(u) as unknown as ProbeSocket
): Promise<boolean> {
  return new Promise((resolve) => {
    let settled = false;
    let socket: ProbeSocket | null = null;
    const finish = (ok: boolean) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      try {
        socket?.close();
      } catch {
        // Already closed.
      }
      resolve(ok);
    };
    const timer = setTimeout(() => finish(false), timeoutMs);
    try {
      socket = createSocket(url);
      socket.onopen = () => finish(true);
      socket.onerror = () => finish(false);
      socket.onclose = () => finish(false);
    } catch {
      finish(false);
    }
  });
}

export type ProbeSocket = {
  onopen: ((event?: unknown) => void) | null;
  onerror: ((event?: unknown) => void) | null;
  onclose: ((event?: unknown) => void) | null;
  close(): void;
};

/** Resolves the source for the page. Only the default bridge is probed; an explicit `?bridge=` is trusted. */
export async function resolveSource(
  search: string,
  tauri: boolean,
  probe: (url: string) => Promise<boolean> = (url) => probeBridge(url)
): Promise<SourceChoice> {
  const choice = sourceFromQuery(search, tauri);
  if (choice !== 'probe') return choice;
  return (await probe(DEFAULT_BRIDGE_URL)) ? { kind: 'bridge', url: DEFAULT_BRIDGE_URL } : { kind: 'mock' };
}
