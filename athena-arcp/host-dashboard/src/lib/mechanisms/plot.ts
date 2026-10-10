import type { Sample } from '../data/types';

export type PlotPoint = { seconds: number; value: number };

/**
 * Keeps numeric samples inside the window ending at `nowMicros`, converted to seconds relative to `nowMicros`
 * (so the newest sample is at 0 and older samples are negative). Non-numeric samples are dropped.
 */
export function windowSeries(samples: readonly Sample[], nowMicros: number, windowSeconds: number): PlotPoint[] {
  const out: PlotPoint[] = [];
  const cutoff = nowMicros - windowSeconds * 1_000_000;
  for (const sample of samples) {
    if (typeof sample.value !== 'number' || sample.timestampMicros < cutoff) continue;
    out.push({ seconds: (sample.timestampMicros - nowMicros) / 1_000_000, value: sample.value });
  }
  return out;
}

export type PlotBox = { width: number; height: number; padding: number };

export type Domain = { min: number; max: number };

/** Y domain over several series, honouring explicit bounds and padding flat ranges. */
export function yDomain(series: readonly (readonly PlotPoint[])[], fixed: { min: number | null; max: number | null }): Domain {
  let min = Infinity;
  let max = -Infinity;
  for (const points of series) {
    for (const p of points) {
      if (p.value < min) min = p.value;
      if (p.value > max) max = p.value;
    }
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    min = 0;
    max = 1;
  }
  if (max - min < 1e-9) {
    min -= 0.5;
    max += 0.5;
  }
  return { min: fixed.min ?? min, max: fixed.max ?? max };
}

/** Maps a series to SVG `points` coordinates. X spans [-windowSeconds, 0]. */
export function toSvgPoints(
  points: readonly PlotPoint[],
  box: PlotBox,
  windowSeconds: number,
  domain: Domain
): string {
  const innerW = box.width - box.padding * 2;
  const innerH = box.height - box.padding * 2;
  const span = domain.max - domain.min || 1;
  return points
    .map((p) => {
      const x = box.padding + ((p.seconds + windowSeconds) / windowSeconds) * innerW;
      const y = box.padding + (1 - (p.value - domain.min) / span) * innerH;
      return `${round(x)},${round(y)}`;
    })
    .join(' ');
}

function round(n: number): number {
  return Math.round(n * 10) / 10;
}
