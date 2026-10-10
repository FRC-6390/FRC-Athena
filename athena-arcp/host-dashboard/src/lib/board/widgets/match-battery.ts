/** Battery voltage reading and sag history for the battery widget. Pure. */

export type BatteryTone = 'ok' | 'warn' | 'fault' | null;

export type BatteryPoint = { seconds: number; volts: number };

export type BatteryReading = {
  volts: number | null;
  /** Lowest voltage in the window, null without samples. */
  min: number | null;
  tone: BatteryTone;
  /** Oldest to newest, `seconds` is relative to the newest sample (<= 0). */
  points: BatteryPoint[];
};

export type BatteryOptions = {
  /** Warn when below this voltage. */
  warnBelow: number;
  /** Fault when below this voltage. */
  faultBelow: number;
  /** Length of the sag history shown. */
  windowSeconds: number;
};

export function batteryTone(volts: number | null, warnBelow: number, faultBelow: number): BatteryTone {
  if (volts === null) return null;
  if (volts < faultBelow) return 'fault';
  if (volts < warnBelow) return 'warn';
  return 'ok';
}

type TimedValue = { timestampMicros: number; value: unknown };

export function batteryReading(samples: readonly TimedValue[], options: BatteryOptions): BatteryReading {
  const numeric = samples
    .map((s) => ({ t: s.timestampMicros, v: typeof s.value === 'bigint' ? Number(s.value) : s.value }))
    .filter((s): s is { t: number; v: number } => typeof s.v === 'number' && Number.isFinite(s.v));
  if (numeric.length === 0) return { volts: null, min: null, tone: null, points: [] };

  const newest = numeric[numeric.length - 1];
  const horizon = newest.t - Math.max(0, options.windowSeconds) * 1_000_000;
  const points = numeric
    .filter((s) => s.t >= horizon)
    .map((s) => ({ seconds: (s.t - newest.t) / 1_000_000, volts: s.v }));
  const min = points.reduce((lowest, p) => Math.min(lowest, p.volts), Number.POSITIVE_INFINITY);
  return {
    volts: newest.v,
    min,
    tone: batteryTone(newest.v, options.warnBelow, options.faultBelow),
    points
  };
}

/** Polyline points for an SVG sparkline of width x height. Empty when fewer than two points. */
export function sparklinePoints(points: readonly BatteryPoint[], width: number, height: number, pad = 2): string {
  if (points.length < 2) return '';
  const first = points[0].seconds;
  const span = Math.max(1e-6, 0 - first);
  let lo = Number.POSITIVE_INFINITY;
  let hi = Number.NEGATIVE_INFINITY;
  for (const p of points) {
    lo = Math.min(lo, p.volts);
    hi = Math.max(hi, p.volts);
  }
  // Keep the chart from flattening when the voltage barely moves.
  if (hi - lo < 0.5) {
    const mid = (hi + lo) / 2;
    lo = mid - 0.25;
    hi = mid + 0.25;
  }
  return points
    .map((p) => {
      const x = pad + ((p.seconds - first) / span) * (width - pad * 2);
      const y = pad + (1 - (p.volts - lo) / (hi - lo)) * (height - pad * 2);
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(' ');
}
