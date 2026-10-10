/**
 * Driver workspace state (F2): derives what the driver station shows from the value store. Pure, so the
 * rules (what counts as an alert, how match phases are read from markers) are unit-tested.
 */
import type { ValueStore } from '../data/store';
import { MARKER_KIND, ROBOT_PATHS, type SignalValue } from '../data/types';
import { CURRENT_NAMES } from '../mechanisms/model';

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

/**
 * One motor in the power split. `name` is the mechanism path without `robot/`, then the device, e.g.
 * `turret/motor`, or the `athena/power/<name>/current` name when the robot publishes one.
 */
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
  /** DriverStation mode (`athena/robot/mode`), null until the robot publishes it. */
  mode: string | null;
  alliance: string | null;
  battery: { volts: number | null };
  power: { totalAmps: number; consumers: PowerConsumer[] };
  camera: { path: string; value: string } | null;
};

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
      const recent = nowMicros - marker.timestampMicros <= FAULT_WINDOW_MICROS;
      if (!recent) continue;
      if (marker.kind === MARKER_KIND.FAULT) {
        alerts.push({
          key: `marker-${marker.timestampMicros}-${marker.message}`,
          level: 'fault',
          text: marker.message || 'Fault'
        });
      } else if (marker.kind === MARKER_KIND.STAGE_CHANGE) {
        alerts.push({
          key: `stage-${marker.timestampMicros}-${marker.message}`,
          level: 'warn',
          text: marker.message || 'Stage changed'
        });
      }
    }
  }

  // Mechanism faults: `<mechanism>/State/Fault` is empty text when there is no fault.
  for (const signal of signals) {
    if (signal.role !== 'state' || signal.localPath !== 'State/Fault') continue;
    const value = store.latestSample(signal.signalId)?.value;
    if (typeof value !== 'string' || value === '') continue;
    alerts.push({ key: signal.path, level: 'fault', text: `${signal.mechanismPath}: ${value}` });
  }

  const mode = textOf(store, ROBOT_PATHS.mode);
  const alliance = textOf(store, ROBOT_PATHS.alliance);
  const markerPhase = matchPhase(store);
  // Without match markers (no FMS or recorder), the DriverStation mode says which period is running.
  const phase: MatchPhase =
    markerPhase.phase === 'idle' ? phaseFromMode(mode) : markerPhase.phase;
  const since = markerPhase.phase === 'idle' ? null : markerPhase.since;

  // The robot's own match clock (seconds left in the period) wins over a countdown from the markers.
  const matchTime = numberOf(valueOf(store, ROBOT_PATHS.matchTime));
  let elapsedSeconds: number | null = null;
  let remainingSeconds: number | null = null;
  if (since !== null && nowMicros !== null) {
    elapsedSeconds = Math.max(0, (nowMicros - since) / 1_000_000);
    if (phase === 'auto') remainingSeconds = Math.max(0, AUTO_SECONDS - elapsedSeconds);
    if (phase === 'teleop') remainingSeconds = Math.max(0, TELEOP_SECONDS - elapsedSeconds);
  }
  if (matchTime !== null && matchTime >= 0 && (phase === 'auto' || phase === 'teleop')) {
    remainingSeconds = matchTime;
    if (elapsedSeconds === null) {
      const length = phase === 'auto' ? AUTO_SECONDS : TELEOP_SECONDS;
      elapsedSeconds = Math.max(0, length - matchTime);
    }
  }

  const voltage = store.descriptorByPath(ROBOT_PATHS.batteryVolts);
  const volts = voltage ? numberOf(store.latestSample(voltage.signalId)?.value) : null;
  const consumers = powerConsumers(store);
  const totalAmps = consumers.reduce((sum, c) => sum + c.amps, 0);
  for (const c of consumers) c.share = totalAmps > 0 ? c.amps / totalAmps : 0;
  consumers.sort((a, b) => b.amps - a.amps || a.name.localeCompare(b.name));

  const camera = signals.find((s) => s.type === 'string' && /camera/i.test(s.path));
  return {
    nowMicros,
    alerts,
    match: { phase, elapsedSeconds, remainingSeconds },
    mode: mode === '' ? null : mode,
    alliance: alliance === '' ? null : alliance,
    battery: { volts },
    power: { totalAmps, consumers },
    camera: camera
      ? { path: camera.path, value: String(store.latestSample(camera.signalId)?.value ?? 'no feed') }
      : null
  };
}

/** Phase from the DriverStation mode. Disabled and unknown mean idle. */
export function phaseFromMode(mode: string | null): MatchPhase {
  if (mode === 'Autonomous') return 'auto';
  if (mode === 'Teleop') return 'teleop';
  return 'idle';
}

function valueOf(store: ValueStore, path: string): SignalValue | undefined {
  const descriptor = store.descriptorByPath(path);
  return descriptor ? store.latestSample(descriptor.signalId)?.value : undefined;
}

function textOf(store: ValueStore, path: string): string {
  const value = valueOf(store, path);
  return value === undefined ? '' : String(value);
}

/**
 * Per-consumer current for the power split. Uses `athena/power/<name>/current` when the robot publishes it;
 * otherwise each motor's supply current from its mechanism (`robot/<mechanism>/Devices/<device>/State/...`).
 */
function powerConsumers(store: ValueStore): PowerConsumer[] {
  const published: PowerConsumer[] = [];
  const fromMechanisms: PowerConsumer[] = [];
  for (const signal of store.allSignals()) {
    const powerMatch = /^athena\/power\/([^/]+)\/current$/.exec(signal.path);
    if (powerMatch) {
      published.push({ name: powerMatch[1], amps: numberOf(store.latestSample(signal.signalId)?.value) ?? 0, share: 0 });
      continue;
    }
    const device = /^Devices\/([^/]+)\/State\/([^/]+)$/.exec(signal.localPath);
    if (device && device[2] === CURRENT_NAMES[0] && signal.mechanismPath.startsWith('robot/')) {
      fromMechanisms.push({
        name: `${signal.mechanismPath.slice('robot/'.length)}/${device[1]}`,
        amps: numberOf(store.latestSample(signal.signalId)?.value) ?? 0,
        share: 0
      });
    }
  }
  return published.length > 0 ? published : fromMechanisms;
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
