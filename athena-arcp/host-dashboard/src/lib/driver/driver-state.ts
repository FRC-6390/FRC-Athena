/**
 * Driver workspace state (F2): derives what the driver station shows from the value store. Pure, so the
 * rules (what counts as an alert, how match phases are read from markers) are unit-tested.
 */
import type { ValueStore } from '../data/store';
import { MARKER_KIND } from '../data/types';

/** Seconds of FRC match time, used for countdowns. */
export const AUTO_SECONDS = 15;
export const TELEOP_SECONDS = 135;

/** Recent faults stay on the banner this long (robot time). */
export const FAULT_WINDOW_MICROS = 15_000_000;

export type DriverAlert = {
  key: string;
  level: 'fault' | 'warn';
  text: string;
};

export type MatchPhase = 'idle' | 'auto' | 'teleop' | 'ended';

export type PowerConsumer = { name: string; amps: number; share: number };

export type DriverState = {
  /** Robot clock: the newest sample timestamp in the store (micros), or null before any data. */
  nowMicros: number | null;
  alerts: DriverAlert[];
  match: {
    phase: MatchPhase;
    /** Seconds since the phase started; null when idle. */
    elapsedSeconds: number | null;
    /** Seconds left in the phase when it has a fixed length; null otherwise. */
    remainingSeconds: number | null;
  };
  mode: string | null;
  battery: { volts: number | null };
  power: { totalAmps: number; consumers: PowerConsumer[] };
  camera: { path: string; value: string } | null;
};

const NORMAL_STAGES = new Set(['', 'NORMAL', 'OK', 'Idle']);

/** Newest robot timestamp across all latest samples. */
export function robotNowMicros(store: ValueStore): number | null {
  let now: number | null = null;
  for (const signal of store.allSignals()) {
    const sample = store.latestSample(signal.signalId);
    if (sample && (now === null || sample.timestampMicros > now)) now = sample.timestampMicros;
  }
  return now;
}

/** Phase from the most recent match marker. Unknown or missing markers mean idle. */
export function matchPhase(store: ValueStore): { phase: MatchPhase; since: number | null } {
  let phase: MatchPhase = 'idle';
  let since: number | null = null;
  for (const marker of store.markers()) {
    switch (marker.kind) {
      case MARKER_KIND.AUTO_START:
        phase = 'auto';
        since = marker.timestampMicros;
        break;
      case MARKER_KIND.TELEOP_START:
        phase = 'teleop';
        since = marker.timestampMicros;
        break;
      case MARKER_KIND.MATCH_END:
        phase = 'ended';
        since = marker.timestampMicros;
        break;
      case MARKER_KIND.BOOT:
        phase = 'idle';
        since = null;
        break;
      default:
        break;
    }
  }
  return { phase, since };
}

export function deriveDriverState(store: ValueStore): DriverState {
  const nowMicros = robotNowMicros(store);
  const signals = store.allSignals();
  const alerts: DriverAlert[] = [];

  if (nowMicros !== null) {
    for (const marker of store.markers()) {
      if (marker.kind !== MARKER_KIND.FAULT) continue;
      if (nowMicros - marker.timestampMicros > FAULT_WINDOW_MICROS) continue;
      alerts.push({
        key: `marker-${marker.timestampMicros}-${marker.message}`,
        level: 'fault',
        text: marker.message || 'Fault'
      });
    }
  }

  for (const signal of signals) {
    if (signal.role !== 'state' || signal.name !== 'stage') continue;
    const value = store.latestSample(signal.signalId)?.value;
    if (typeof value !== 'string' || NORMAL_STAGES.has(value)) continue;
    alerts.push({ key: signal.path, level: 'warn', text: `${signal.mechanismPath}: ${value}` });
  }

  const { phase, since } = matchPhase(store);
  let elapsedSeconds: number | null = null;
  let remainingSeconds: number | null = null;
  if (since !== null && nowMicros !== null) {
    elapsedSeconds = Math.max(0, (nowMicros - since) / 1_000_000);
    if (phase === 'auto') remainingSeconds = Math.max(0, AUTO_SECONDS - elapsedSeconds);
    if (phase === 'teleop') remainingSeconds = Math.max(0, TELEOP_SECONDS - elapsedSeconds);
  }

  const modeSignal = store.descriptorByPath('athena/mode/operator');
  const mode = modeSignal ? String(store.latestSample(modeSignal.signalId)?.value ?? '') : null;

  const voltage = store.descriptorByPath('athena/power/battery/voltage');
  const volts = voltage ? numberOf(store.latestSample(voltage.signalId)?.value) : null;
  const consumers: PowerConsumer[] = [];
  for (const signal of signals) {
    const match = /^athena\/power\/([^/]+)\/current$/.exec(signal.path);
    if (!match) continue;
    consumers.push({ name: match[1], amps: numberOf(store.latestSample(signal.signalId)?.value) ?? 0, share: 0 });
  }
  const totalAmps = consumers.reduce((sum, c) => sum + c.amps, 0);
  for (const c of consumers) c.share = totalAmps > 0 ? c.amps / totalAmps : 0;
  consumers.sort((a, b) => b.amps - a.amps);

  const camera = signals.find((s) => s.type === 'string' && /camera/i.test(s.path));
  return {
    nowMicros,
    alerts,
    match: { phase, elapsedSeconds, remainingSeconds },
    mode: mode === null || mode === '' ? null : mode,
    battery: { volts },
    power: { totalAmps, consumers },
    camera: camera
      ? { path: camera.path, value: String(store.latestSample(camera.signalId)?.value ?? 'no feed') }
      : null
  };
}

function numberOf(value: unknown): number | null {
  if (typeof value === 'number') return value;
  if (typeof value === 'bigint') return Number(value);
  return null;
}

/** mm:ss for the match clock. */
export function formatClock(seconds: number | null): string {
  if (seconds === null) return '--:--';
  const total = Math.ceil(seconds);
  return `${Math.floor(total / 60)}:${String(total % 60).padStart(2, '0')}`;
}
