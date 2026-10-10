/**
 * Alerts feed rules: recent fault, stage and check markers plus boolean fault signals, newest first.
 * Dismissals are local: a dismissed marker stays hidden; a fault signal re-arms once it clears.
 */
import { MARKER_KIND } from '../../data/types';

export type AlertLevel = 'fault' | 'warn' | 'info';

export type Alert = {
  key: string;
  level: AlertLevel;
  text: string;
  /** Robot clock, or null for signals that have no sample time. */
  timestampMicros: number | null;
};

export type MarkerInput = { timestampMicros: number; kind: number; message: string; label: string };

export type FaultSignalInput = { path: string; active: boolean; timestampMicros: number | null; name: string };

export type AlertInput = {
  markers: readonly MarkerInput[];
  faultSignals: readonly FaultSignalInput[];
  dismissed: ReadonlySet<string>;
  limit: number;
  showWarnings: boolean;
};

export type AlertResult = {
  alerts: Alert[];
  /** Dismissed keys that still apply; the rest are dropped so faults can re-arm. */
  dismissedNext: Set<string>;
};

export function markerKey(marker: MarkerInput): string {
  return `marker:${marker.timestampMicros}:${marker.kind}:${marker.message}`;
}

export function signalKey(path: string): string {
  return `signal:${path}`;
}

const MARKER_LEVEL: Record<number, AlertLevel> = {
  [MARKER_KIND.FAULT]: 'fault',
  [MARKER_KIND.STAGE_CHANGE]: 'warn',
  [MARKER_KIND.CHECK]: 'info'
};

export function buildAlerts(input: AlertInput): AlertResult {
  const alerts: Alert[] = [];
  const liveKeys = new Set<string>();

  for (const marker of input.markers) {
    const level = MARKER_LEVEL[marker.kind];
    if (!level) continue;
    if (level === 'warn' && !input.showWarnings) continue;
    const key = markerKey(marker);
    liveKeys.add(key);
    alerts.push({ key, level, text: marker.message || marker.label, timestampMicros: marker.timestampMicros });
  }

  for (const signal of input.faultSignals) {
    const key = signalKey(signal.path);
    if (!signal.active) continue;
    liveKeys.add(key);
    alerts.push({ key, level: 'fault', text: signal.name, timestampMicros: signal.timestampMicros });
  }

  const dismissedNext = new Set([...input.dismissed].filter((key) => liveKeys.has(key)));
  const visible = alerts
    .filter((a) => !dismissedNext.has(a.key))
    .sort((a, b) => (b.timestampMicros ?? Number.POSITIVE_INFINITY) - (a.timestampMicros ?? Number.POSITIVE_INFINITY));
  return { alerts: visible.slice(0, Math.max(0, input.limit)), dismissedNext };
}

/** Age label relative to the newest robot time, e.g. `T-12.4 s`. */
export function alertAge(timestampMicros: number | null, nowMicros: number | null): string {
  if (timestampMicros === null || nowMicros === null) return '';
  const seconds = Math.max(0, (nowMicros - timestampMicros) / 1_000_000);
  return seconds < 60 ? `${seconds.toFixed(1)} s ago` : `${Math.floor(seconds / 60)} m ago`;
}
