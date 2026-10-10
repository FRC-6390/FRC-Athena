/**
 * Time-series helpers for the chart widgets: windowing, pairing, decimation, rolling statistics and
 * axis domains. Pure functions over the store's samples (no DOM, no Svelte).
 */
import type { Sample, SignalValue } from '../../data/types';

export type TimedValue = { t: number; v: number };
export type XYPoint = { x: number; y: number; t: number };

/** Numeric view of a signal value, or null for values that cannot be plotted. */
export function numericOf(value: SignalValue | undefined): number | null {
  if (typeof value === 'number') return Number.isFinite(value) ? value : null;
  if (typeof value === 'bigint') return Number(value);
  return null;
}

/**
 * Numeric samples inside the window ending at `nowMicros`. `t` is seconds relative to `nowMicros`
 * (newest is 0, older is negative). Samples are returned oldest first.
 */
export function windowedNumeric(samples: readonly Sample[], nowMicros: number, windowSeconds: number): TimedValue[] {
  const out: TimedValue[] = [];
  const cutoff = nowMicros - windowSeconds * 1_000_000;
  for (const sample of samples) {
    if (sample.timestampMicros < cutoff) continue;
    const v = numericOf(sample.value);
    if (v === null) continue;
    out.push({ t: (sample.timestampMicros - nowMicros) / 1_000_000, v });
  }
  return out;
}

/** Newest timestamp across several sample lists (0 when all are empty). */
export function newestTimestamp(lists: readonly (readonly Sample[])[]): number {
  let newest = 0;
  for (const list of lists) {
    const last = list[list.length - 1];
    if (last && last.timestampMicros > newest) newest = last.timestampMicros;
  }
  return newest;
}

/**
 * Pairs x and y samples by time. Each x sample inside the window is matched with the latest y sample at or
 * before it (zero-order hold), and skipped when no y sample is that recent. Both lists must be oldest first.
 */
export function alignPairs(
  xs: readonly Sample[],
  ys: readonly Sample[],
  nowMicros: number,
  windowSeconds: number
): XYPoint[] {
  const out: XYPoint[] = [];
  const cutoff = nowMicros - windowSeconds * 1_000_000;
  let j = -1;
  let heldY: number | null = null;
  for (const x of xs) {
    while (j + 1 < ys.length && ys[j + 1].timestampMicros <= x.timestampMicros) {
      j += 1;
      heldY = numericOf(ys[j].value);
    }
    if (x.timestampMicros < cutoff || heldY === null) continue;
    const xv = numericOf(x.value);
    if (xv === null) continue;
    out.push({ x: xv, y: heldY, t: (x.timestampMicros - nowMicros) / 1_000_000 });
  }
  return out;
}

/** Keeps at most `max` items by striding, always keeping the newest (last) item. */
export function decimateStride<T>(items: readonly T[], max: number): T[] {
  if (max <= 0) return [];
  if (items.length <= max) return items.slice();
  const out: T[] = [];
  const step = (items.length - 1) / (max - 1);
  for (let i = 0; i < max - 1; i += 1) out.push(items[Math.floor(i * step)]);
  out.push(items[items.length - 1]);
  return out;
}

export type Column = { col: number; min: number; max: number; last: number };

/**
 * Buckets timed values into `columns` equal time slices over [t0, t1] and keeps min, max and last per
 * non-empty slice. Output size is bounded by `columns`, so long histories draw in constant work.
 */
export function minMaxColumns(values: readonly TimedValue[], columns: number, t0: number, t1: number): Column[] {
  const out: Column[] = [];
  if (columns <= 0 || !(t1 > t0)) return out;
  const scale = columns / (t1 - t0);
  let current: Column | null = null;
  for (const p of values) {
    const col = Math.min(columns - 1, Math.max(0, Math.floor((p.t - t0) * scale)));
    if (current && current.col === col) {
      if (p.v < current.min) current.min = p.v;
      if (p.v > current.max) current.max = p.v;
      current.last = p.v;
    } else {
      current = { col, min: p.v, max: p.v, last: p.v };
      out.push(current);
    }
  }
  return out;
}

export type Domain = { min: number; max: number };

/**
 * Padded axis domain over finite values. Explicit bounds win. Flat or tiny ranges widen to `minSpan` around
 * their middle so a constant signal does not collapse onto one pixel row.
 */
export function paddedDomain(
  min: number,
  max: number,
  options: { minSpan: number; fixedMin?: number | null; fixedMax?: number | null; padding?: number }
): Domain {
  let lo = Number.isFinite(min) ? min : 0;
  let hi = Number.isFinite(max) ? max : 1;
  const span = hi - lo;
  const mid = (hi + lo) / 2;
  if (span < options.minSpan) {
    lo = mid - options.minSpan / 2;
    hi = mid + options.minSpan / 2;
  } else {
    const pad = span * (options.padding ?? 0.08);
    lo -= pad;
    hi += pad;
  }
  const fixedMin = options.fixedMin ?? null;
  const fixedMax = options.fixedMax ?? null;
  return { min: fixedMin ?? lo, max: fixedMax ?? hi };
}

/** Square-ish XY domain: with `equal`, the shorter axis is widened so one unit is the same length on both. */
export function xyDomain(points: readonly XYPoint[], equal: boolean): { x: Domain; y: Domain } {
  let x0 = Infinity;
  let x1 = -Infinity;
  let y0 = Infinity;
  let y1 = -Infinity;
  for (const p of points) {
    if (p.x < x0) x0 = p.x;
    if (p.x > x1) x1 = p.x;
    if (p.y < y0) y0 = p.y;
    if (p.y > y1) y1 = p.y;
  }
  let x = paddedDomain(x0, x1, { minSpan: 1e-3 });
  let y = paddedDomain(y0, y1, { minSpan: 1e-3 });
  if (equal) {
    const sx = x.max - x.min;
    const sy = y.max - y.min;
    if (sx > sy) y = { min: (y.min + y.max) / 2 - sx / 2, max: (y.min + y.max) / 2 + sx / 2 };
    else x = { min: (x.min + x.max) / 2 - sy / 2, max: (x.min + x.max) / 2 + sy / 2 };
  }
  return { x, y };
}

export type RollingStats = {
  count: number;
  min: number;
  max: number;
  mean: number;
  /** Population standard deviation. */
  std: number;
  /** Samples per second across the observed span (0 with fewer than two samples). */
  rateHz: number;
  /** Counts per equal-width bin across [min, max]. Empty when `bins` is 0 or the range is flat. */
  histogram: number[];
};

/**
 * Min, max, mean, standard deviation and sample rate of the numeric samples inside the window. Uses a single
 * Welford pass; the histogram takes one more pass over the same samples. Returns null with no numeric data.
 */
export function rollingStats(
  samples: readonly Sample[],
  nowMicros: number,
  windowSeconds: number,
  bins = 12
): RollingStats | null {
  const cutoff = nowMicros - windowSeconds * 1_000_000;
  let count = 0;
  let mean = 0;
  let m2 = 0;
  let min = Infinity;
  let max = -Infinity;
  let first = Infinity;
  let last = -Infinity;
  for (const sample of samples) {
    if (sample.timestampMicros < cutoff) continue;
    const v = numericOf(sample.value);
    if (v === null) continue;
    count += 1;
    const delta = v - mean;
    mean += delta / count;
    m2 += delta * (v - mean);
    if (v < min) min = v;
    if (v > max) max = v;
    if (sample.timestampMicros < first) first = sample.timestampMicros;
    if (sample.timestampMicros > last) last = sample.timestampMicros;
  }
  if (count === 0) return null;
  const spanSeconds = (last - first) / 1_000_000;
  const rateHz = count > 1 && spanSeconds > 0 ? (count - 1) / spanSeconds : 0;
  const histogram = new Array<number>(bins > 0 && max > min ? bins : 0).fill(0);
  if (histogram.length > 0) {
    const scale = histogram.length / (max - min);
    for (const sample of samples) {
      if (sample.timestampMicros < cutoff) continue;
      const v = numericOf(sample.value);
      if (v === null) continue;
      const bin = Math.min(histogram.length - 1, Math.floor((v - min) * scale));
      histogram[bin] += 1;
    }
  }
  return { count, min, max, mean, std: Math.sqrt(Math.max(0, m2 / count)), rateHz, histogram };
}

/** About `count` evenly spaced round values (1, 2, 2.5, 5 × 10^n) covering `domain`, for axis labels. */
export function niceTicks(domain: Domain, count = 4): number[] {
  const span = domain.max - domain.min;
  if (!(span > 0) || !Number.isFinite(span)) return [];
  const raw = span / Math.max(1, count);
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * magnitude).find((s) => s >= raw) ?? raw;
  const ticks: number[] = [];
  for (let v = Math.ceil(domain.min / step) * step; v <= domain.max + step * 1e-9; v += step) {
    ticks.push(Math.abs(v) < step * 1e-9 ? 0 : Number(v.toPrecision(12)));
  }
  return ticks;
}
