/**
 * Plot data preparation for the Analysis plots. Pure functions: slice a visible window out of a track,
 * decimate it to roughly one point per device pixel (LTTB), and map it to WebGL clip space in double
 * precision before the Float32 conversion (robot micros would lose precision in float32).
 */
import { lttb, type Point } from './lttb';
import { lastIndexAtOrBefore } from './replay';

export type YRange = { min: number; max: number };

/**
 * Points of a numeric track inside [fromMicros, toMicros], with the sample in effect at `fromMicros` included
 * so a line enters the window at the right height. x is seconds relative to `originMicros`.
 */
export function windowPoints(
  t: readonly number[],
  v: readonly number[],
  fromMicros: number,
  toMicros: number,
  originMicros: number
): Point[] {
  const out: Point[] = [];
  const first = lastIndexAtOrBefore(t, fromMicros);
  let i = first < 0 ? 0 : first;
  for (; i < t.length; i++) {
    if (t[i] > toMicros) break;
    out.push({ x: (t[i] - originMicros) / 1_000_000, y: v[i] });
  }
  return out;
}

/** Decimates to `maxPoints` (typically the plot width in device pixels). */
export function decimate(points: Point[], maxPoints: number): Point[] {
  return lttb(points, Math.max(3, maxPoints));
}

/** Min/max of y over the points, widened so a flat signal still draws mid-plot. */
export function yRangeOf(points: readonly Point[]): YRange {
  if (points.length === 0) return { min: -1, max: 1 };
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (const p of points) {
    if (p.y < min) min = p.y;
    if (p.y > max) max = p.y;
  }
  if (min === max) {
    const pad = Math.abs(min) * 0.1 || 1;
    return { min: min - pad, max: max + pad };
  }
  const pad = (max - min) * 0.05;
  return { min: min - pad, max: max + pad };
}

/**
 * Interleaved clip-space vertices (x, y pairs in [-1, 1]) for a window [xMin, xMax] x [y.min, y.max].
 * Values outside the window are clamped to its edge.
 */
export function toClipSpace(points: readonly Point[], xMin: number, xMax: number, y: YRange): Float32Array {
  const out = new Float32Array(points.length * 2);
  const xSpan = xMax - xMin || 1;
  const ySpan = y.max - y.min || 1;
  for (let i = 0; i < points.length; i++) {
    const cx = ((points[i].x - xMin) / xSpan) * 2 - 1;
    const cy = ((points[i].y - y.min) / ySpan) * 2 - 1;
    out[i * 2] = Math.max(-1, Math.min(1, cx));
    out[i * 2 + 1] = Math.max(-1, Math.min(1, cy));
  }
  return out;
}
