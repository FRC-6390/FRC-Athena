/**
 * Pure helpers for the Values pack: gauge and bar geometry, alarm tones, sparklines, deltas and layouts.
 */
import type { Sample, SignalValue } from '../../data/types';
import { finiteSetting } from './controls-numeric';

export type AlertRules = {
  warn: unknown;
  fault: unknown;
  /** `high`: alarm when the value rises to a threshold. `low`: alarm when it falls to one. */
  alertWhen: 'high' | 'low';
};

/** Tone for a number against warn / fault thresholds. Null when no threshold is crossed (neutral). */
export function alertTone(value: number | null | undefined, rules: AlertRules): 'warn' | 'fault' | null {
  if (value === null || value === undefined || !Number.isFinite(value)) return null;
  const warn = finiteSetting(rules.warn);
  const fault = finiteSetting(rules.fault);
  const crossed = (threshold: number | null) =>
    threshold !== null && (rules.alertWhen === 'low' ? value <= threshold : value >= threshold);
  if (crossed(fault)) return 'fault';
  if (crossed(warn)) return 'warn';
  return null;
}

/** Position of `value` between min and max as 0..1 (clamped). Degenerate ranges give 0. */
export function fraction(value: number | null | undefined, min: number, max: number): number {
  if (value === null || value === undefined || !Number.isFinite(value)) return 0;
  if (max === min) return 0;
  return Math.min(1, Math.max(0, (value - min) / (max - min)));
}

/** Angle in degrees for a gauge sweep from -120 (min) to +120 (max). */
export function gaugeAngle(value: number | null | undefined, min: number, max: number): number {
  return -120 + fraction(value, min, max) * 240;
}

/** Circular arc path between two fractions of the gauge sweep, for threshold bands. */
export function arcPath(cx: number, cy: number, r: number, fromFrac: number, toFrac: number): string {
  const a0 = ((-120 + fromFrac * 240 - 90) * Math.PI) / 180;
  const a1 = ((-120 + toFrac * 240 - 90) * Math.PI) / 180;
  const x0 = cx + r * Math.cos(a0);
  const y0 = cy + r * Math.sin(a0);
  const x1 = cx + r * Math.cos(a1);
  const y1 = cy + r * Math.sin(a1);
  const large = toFrac - fromFrac > 0.5 ? 1 : 0;
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 ${large} 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`;
}

/** Numeric samples inside the last `seconds`, oldest first. */
export function numericWindow(samples: readonly Sample[], nowMicros: number, seconds: number): { t: number; v: number }[] {
  const cutoff = nowMicros - seconds * 1_000_000;
  const out: { t: number; v: number }[] = [];
  for (const s of samples) {
    if (typeof s.value !== 'number' || !Number.isFinite(s.value) || s.timestampMicros < cutoff) continue;
    out.push({ t: s.timestampMicros, v: s.value });
  }
  return out;
}

/** Change from the oldest sample in the window to the newest. Null with fewer than two samples. */
export function windowDelta(samples: readonly Sample[], nowMicros: number, seconds: number): number | null {
  const pts = numericWindow(samples, nowMicros, seconds);
  if (pts.length < 2) return null;
  return pts[pts.length - 1].v - pts[0].v;
}

/**
 * SVG path for a sparkline in a `width` x `height` box. A flat series is drawn mid-height, and x spans the
 * window so a short history stays at its real timescale.
 */
export function sparkPath(samples: readonly Sample[], nowMicros: number, seconds: number, width: number, height: number): string {
  const pts = numericWindow(samples, nowMicros, seconds);
  if (pts.length < 2) return '';
  let lo = Infinity;
  let hi = -Infinity;
  for (const p of pts) {
    if (p.v < lo) lo = p.v;
    if (p.v > hi) hi = p.v;
  }
  const span = hi - lo;
  const cutoff = nowMicros - seconds * 1_000_000;
  return pts
    .map((p, i) => {
      const x = ((p.t - cutoff) / (seconds * 1_000_000)) * width;
      const y = span === 0 ? height / 2 : height - ((p.v - lo) / span) * height;
      return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)} ${y.toFixed(1)}`;
    })
    .join(' ');
}

export type ToneRules = {
  warnAbove: unknown;
  faultAbove: unknown;
  warnBelow: unknown;
  faultBelow: unknown;
  /** Tone for a true boolean and for a false boolean. */
  trueTone: string;
  falseTone: string;
};

/**
 * Row tone for a status list. Missing data is `stale`, strings are neutral (null), and numbers use the
 * above / below thresholds (fault wins over warn); numbers with no thresholds are neutral too.
 */
export function rowTone(value: SignalValue | undefined, rules: ToneRules): string | null {
  if (value === undefined) return 'stale';
  if (typeof value === 'boolean') return value ? rules.trueTone : rules.falseTone;
  const n = typeof value === 'number' ? value : typeof value === 'bigint' ? Number(value) : null;
  if (n === null || !Number.isFinite(n)) return null;
  const fa = finiteSetting(rules.faultAbove);
  const wa = finiteSetting(rules.warnAbove);
  const fb = finiteSetting(rules.faultBelow);
  const wb = finiteSetting(rules.warnBelow);
  // Without thresholds a number carries no status, so it gets no light.
  if (fa === null && wa === null && fb === null && wb === null) return null;
  if ((fa !== null && n >= fa) || (fb !== null && n <= fb)) return 'fault';
  if ((wa !== null && n >= wa) || (wb !== null && n <= wb)) return 'warn';
  return 'ok';
}

/** Column count for a grid of `count` lights. A positive setting wins; auto is roughly square. */
export function matrixColumns(count: number, setting: unknown): number {
  const fixed = finiteSetting(setting);
  if (fixed !== null && fixed >= 1) return Math.floor(fixed);
  return Math.max(1, Math.ceil(Math.sqrt(count)));
}

/** `drive_motor_temp` or `shotDistanceMeters` to `Drive motor temp` / `Shot distance meters`. */
export function humanLabel(name: string): string {
  const spaced = name.replace(/_/g, ' ').replace(/([a-z0-9])([A-Z])/g, '$1 $2').replace(/\s+/g, ' ').trim().toLowerCase();
  if (spaced === '') return name;
  return spaced.charAt(0).toUpperCase() + spaced.slice(1);
}
