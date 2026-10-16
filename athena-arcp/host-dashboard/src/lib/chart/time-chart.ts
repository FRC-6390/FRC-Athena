/**
 * Data preparation for the time-series chart (src/ui/TimeChart.svelte). Pure functions over the store's
 * samples: windowing, aligning several series on one time axis (gaps are null), the visible y domain and
 * axis labels. No DOM, no uPlot runtime, so it is testable with `bun test`.
 */
import type { Sample } from '../data/types';
import { numericOf } from '../board/widgets/robot-series';

export type ChartInput = { id: string; samples: readonly Sample[] };
/** One line on the chart: its samples plus how it is drawn. `color` may be a `var(--token)`. */
export type ChartSeries = ChartInput & { label: string; color: string; dashed?: boolean };
export type Range = { min: number; max: number };
/** Column 0 is x in seconds (absolute); the rest are y values per series, null where that series has no sample. */
export type ChartData = [number[], ...(number | null)[][]];
export type BuiltChart = { data: ChartData; newestSeconds: number };

/** Index of the first sample at or after `micros` (samples are oldest first). */
function firstAtOrAfter(samples: readonly Sample[], micros: number): number {
  let lo = 0;
  let hi = samples.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (samples[mid].timestampMicros < micros) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}

/**
 * Builds the aligned table for every series. The window ends at the newest sample of any series and starts
 * `windowSeconds` earlier, or earlier still when `zoomMinSeconds` asks for older data.
 */
export function buildChartData(inputs: readonly ChartInput[], windowSeconds: number, zoomMinSeconds: number | null = null): BuiltChart {
  let newestMicros = 0;
  for (const input of inputs) {
    const last = input.samples[input.samples.length - 1];
    if (last && last.timestampMicros > newestMicros) newestMicros = last.timestampMicros;
  }
  const newestSeconds = newestMicros / 1_000_000;
  const startSeconds = Math.min(newestSeconds - windowSeconds, zoomMinSeconds ?? Infinity);
  const startMicros = startSeconds * 1_000_000;

  // Numeric samples inside the window, per series, as integer microseconds and values.
  const columns = inputs.map((input) => {
    const times: number[] = [];
    const values: number[] = [];
    for (let i = firstAtOrAfter(input.samples, startMicros); i < input.samples.length; i += 1) {
      const sample = input.samples[i];
      const v = numericOf(sample.value);
      if (v === null) continue;
      times.push(sample.timestampMicros);
      values.push(v);
    }
    return { times, values };
  });

  // Union of all sample times, sorted and without duplicates.
  const all: number[] = [];
  for (const column of columns) for (const t of column.times) all.push(t);
  all.sort((a, b) => a - b);
  const unique: number[] = [];
  for (const t of all) if (unique.length === 0 || unique[unique.length - 1] !== t) unique.push(t);

  const xs = unique.map((t) => t / 1_000_000);
  const ys: (number | null)[][] = columns.map((column) => {
    const out: (number | null)[] = new Array(unique.length).fill(null);
    let j = 0;
    for (let i = 0; i < unique.length; i += 1) {
      while (j < column.times.length && column.times[j] < unique[i]) j += 1;
      if (j < column.times.length && column.times[j] === unique[i]) out[i] = column.values[j];
    }
    return out;
  });

  if (xs.length === 0) {
    // Nothing to draw yet: keep an empty two-point axis so the frame and gridlines still render.
    return {
      data: [[newestSeconds - windowSeconds, newestSeconds], ...inputs.map(() => [null, null])] as ChartData,
      newestSeconds
    };
  }
  return { data: [xs, ...ys] as ChartData, newestSeconds };
}

/** Visible x range: the zoom when set, otherwise the trailing window ending at `newestSeconds`. */
export function visibleX(newestSeconds: number, windowSeconds: number, zoom: Range | null): Range {
  return zoom ?? { min: newestSeconds - windowSeconds, max: newestSeconds };
}

/**
 * Y domain over the visible x range for the series that are not hidden. Auto ranges get a small margin and
 * a minimum span so a steady value stays flat instead of filling the plot. Fixed bounds win over auto ones.
 */
export function yDomainFor(
  data: ChartData,
  x: Range,
  hidden: readonly boolean[],
  fixed: { min: number | null; max: number | null }
): Range {
  let lo = Infinity;
  let hi = -Infinity;
  const xs = data[0];
  for (let i = 0; i < xs.length; i += 1) {
    if (xs[i] < x.min || xs[i] > x.max) continue;
    for (let s = 1; s < data.length; s += 1) {
      if (hidden[s - 1]) continue;
      const v = data[s][i];
      if (v === null || v === undefined) continue;
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
    lo = 0;
    hi = 1;
  } else {
    const span = hi - lo;
    const mid = (hi + lo) / 2;
    const minSpan = Math.max(Math.abs(mid) * 0.1, 0.5);
    if (span < minSpan) {
      lo = mid - minSpan / 2;
      hi = mid + minSpan / 2;
    } else {
      const pad = span * 0.08;
      lo -= pad;
      hi += pad;
    }
  }
  if (fixed.min !== null) lo = fixed.min;
  if (fixed.max !== null) hi = fixed.max;
  if (!(hi > lo)) hi = lo + 1;
  return { min: lo, max: hi };
}

/** Axis label for a time offset in seconds from the newest sample: `now` at 0, otherwise `-12 s`. */
export function relativeLabel(offsetSeconds: number): string {
  if (Math.abs(offsetSeconds) < 0.05) return 'now';
  return `${Math.round(offsetSeconds * 10) / 10} s`;
}

/**
 * Value of one series at row `index`, held from the last earlier sample when that row has none (so a
 * readout under the cursor always shows the value in effect at that time). Null when there is none yet.
 */
export function valueHeldAt(column: readonly (number | null)[], index: number): number | null {
  for (let i = Math.min(index, column.length - 1); i >= 0; i -= 1) {
    const v = column[i];
    if (v !== null && v !== undefined) return v;
  }
  return null;
}
