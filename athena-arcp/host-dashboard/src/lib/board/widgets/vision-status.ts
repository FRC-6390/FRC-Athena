/**
 * Pure helpers for the Vision status widget and the vision driver alert: grouping `athena/vision/<camera>/<field>`
 * signals by camera, state tone, the "not accepted for a while" warning and row formatting.
 */
import { ROBOT_PATHS, toNumber, type Sample, type SignalValue } from '../../data/types';

/** Path prefix of the vision telemetry the robot publishes per camera. */
export const VISION_PREFIX = ROBOT_PATHS.visionPrefix;

/** Fields the robot publishes per camera (TimeSync may be absent). */
export const VISION_FIELDS = [
  'State',
  'Reason',
  'AcceptedPerSecond',
  'RejectedPerSecond',
  'FrameAgeMs',
  'TagCount',
  'SecondsSinceAccepted',
  'TimeSync'
] as const;

/** A camera with no accepted pose for longer than this, while frames still arrive, is a warning. */
export const NOT_ACCEPTED_WARN_SECONDS = 2;
/** Frames count as arriving when their age is under this many milliseconds. */
export const FRAMES_ARRIVING_MS = 1000;
/** UNSYNCED or STALE for longer than this while enabled raises a driver alert. */
export const SUSTAINED_ALERT_SECONDS = 2;

export type VisionTone = 'ok' | 'info' | 'warn' | 'stale';

/** Camera name and field from a vision path, e.g. `athena/vision/front/State` gives `front` and `State`. */
export function visionPath(path: string): { camera: string; field: string } | null {
  if (!path.startsWith(VISION_PREFIX)) return null;
  const rest = path.slice(VISION_PREFIX.length);
  const cut = rest.lastIndexOf('/');
  if (cut <= 0 || cut === rest.length - 1) return null;
  return { camera: rest.slice(0, cut), field: rest.slice(cut + 1) };
}

export type VisionCamera<T extends { path: string }> = {
  name: string;
  /** Signals of this camera keyed by field name. */
  signals: Record<string, T>;
};

/** Groups vision signals by camera, sorted by camera name. Paths outside the prefix are skipped. */
export function groupCameras<T extends { path: string }>(signals: readonly T[]): VisionCamera<T>[] {
  const cameras = new Map<string, VisionCamera<T>>();
  for (const signal of signals) {
    const parsed = visionPath(signal.path);
    if (!parsed) continue;
    let camera = cameras.get(parsed.camera);
    if (!camera) {
      camera = { name: parsed.camera, signals: {} };
      cameras.set(parsed.camera, camera);
    }
    camera.signals[parsed.field] = signal;
  }
  return [...cameras.values()].sort((a, b) => a.name.localeCompare(b.name));
}

/** Tone for a published State. Missing or empty is stale; unrecognised text is info. */
export function visionTone(state: SignalValue | undefined): VisionTone {
  if (typeof state !== 'string' || state.trim() === '') return 'stale';
  switch (state.trim().toUpperCase()) {
    case 'OK':
      return 'ok';
    case 'RECOVERING':
      return 'info';
    case 'REJECTED':
    case 'UNSYNCED':
      return 'warn';
    case 'STALE':
    case 'NO_TARGETS':
      return 'stale';
    default:
      return 'info';
  }
}

/**
 * True when the camera has not had an accepted pose for more than the warning time while frames still arrive
 * (a camera that is not getting frames is STALE, which already says so).
 */
export function notAcceptedWarning(secondsSinceAccepted: number | null, frameAgeMs: number | null): boolean {
  if (secondsSinceAccepted === null || frameAgeMs === null) return false;
  return secondsSinceAccepted > NOT_ACCEPTED_WARN_SECONDS && frameAgeMs < FRAMES_ARRIVING_MS;
}

/** Number from a signal value (i64 bigints included); null when missing or not finite. */
export function numberOf(value: SignalValue | undefined): number | null {
  if (typeof value === 'number') return Number.isFinite(value) ? value : null;
  if (typeof value === 'bigint') return toNumber(value);
  return null;
}

export type VisionRow = {
  camera: string;
  /** State text, or `--` when the signal is missing. */
  state: string;
  tone: VisionTone;
  /** Reason text (empty when OK), or `--` when the signal is missing. */
  reason: string;
  tags: string;
  acceptedPerSecond: string;
  rejectedPerSecond: string;
  frameAge: string;
  sinceAccepted: string;
  /** Accepted pose is overdue while frames arrive. */
  stalled: boolean;
  /** TimeSync is published and false. */
  clockNotSynced: boolean;
};

export function formatRate(value: number | null): string {
  return value === null ? '--' : value.toFixed(1);
}

export function formatMs(value: number | null): string {
  return value === null ? '--' : `${Math.round(value)} ms`;
}

export function formatSeconds(value: number | null): string {
  return value === null ? '--' : `${value.toFixed(1)} s`;
}

/**
 * One camera's row. `read` gives the latest value of a field (undefined when missing); every missing field
 * renders as `--`.
 */
export function visionRow(camera: string, read: (field: string) => SignalValue | undefined): VisionRow {
  const stateValue = read('State');
  const state = typeof stateValue === 'string' && stateValue.trim() !== '' ? stateValue.trim() : '--';
  const reasonValue = read('Reason');
  const reason = typeof reasonValue === 'string' ? reasonValue : '--';
  const sinceValue = numberOf(read('SecondsSinceAccepted'));
  const frameValue = numberOf(read('FrameAgeMs'));
  const stalled = notAcceptedWarning(sinceValue, frameValue);
  const tagValue = numberOf(read('TagCount'));
  return {
    camera,
    state,
    tone: stalled ? 'warn' : visionTone(stateValue),
    reason,
    tags: tagValue === null ? '--' : String(Math.round(tagValue)),
    acceptedPerSecond: formatRate(numberOf(read('AcceptedPerSecond'))),
    rejectedPerSecond: formatRate(numberOf(read('RejectedPerSecond'))),
    frameAge: formatMs(frameValue),
    sinceAccepted: formatSeconds(sinceValue),
    stalled,
    clockNotSynced: read('TimeSync') === false
  };
}

/**
 * Driver alert text when a camera has been UNSYNCED or STALE for more than the alert time, judged from its
 * State history (oldest first). Null when the newest state is fine or the run is too short. The caller decides
 * whether the robot is enabled.
 */
export function visionAlertText(camera: string, samples: readonly Sample[], nowMicros: number | null): string | null {
  if (nowMicros === null || samples.length === 0) return null;
  const last = samples[samples.length - 1];
  if (typeof last.value !== 'string') return null;
  const state = last.value.trim().toUpperCase();
  if (state !== 'UNSYNCED' && state !== 'STALE') return null;
  let start = last.timestampMicros;
  for (let i = samples.length - 2; i >= 0; i--) {
    const earlier = samples[i].value;
    if (typeof earlier !== 'string' || earlier.trim().toUpperCase() !== state) break;
    start = samples[i].timestampMicros;
  }
  if ((nowMicros - start) / 1_000_000 <= SUSTAINED_ALERT_SECONDS) return null;
  return `${camera} camera: ${state}`;
}
