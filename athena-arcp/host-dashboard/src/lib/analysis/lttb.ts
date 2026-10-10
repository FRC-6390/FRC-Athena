/**
 * Largest-Triangle-Three-Buckets decimation (Steinarsson 2013). Keeps the first and last points and, for
 * each bucket, the point forming the largest triangle with its neighbours, so peaks survive. Input must be
 * sorted by x. Output is a new array; input is never mutated.
 */
export type Point = { x: number; y: number };

export function lttb(points: readonly Point[], threshold: number): Point[] {
  const n = points.length;
  const target = Math.floor(threshold);
  if (!Number.isFinite(target) || target >= n || target < 3) return points.slice();

  const out: Point[] = [points[0]];
  const bucketSize = (n - 2) / (target - 2);
  let anchor = 0;

  for (let i = 0; i < target - 2; i++) {
    // Average of the next bucket is the third vertex of each candidate triangle.
    const nextStart = Math.floor((i + 1) * bucketSize) + 1;
    const nextEnd = Math.min(Math.floor((i + 2) * bucketSize) + 1, n);
    let avgX = 0;
    let avgY = 0;
    for (let j = nextStart; j < nextEnd; j++) {
      avgX += points[j].x;
      avgY += points[j].y;
    }
    const count = Math.max(1, nextEnd - nextStart);
    avgX /= count;
    avgY /= count;

    const start = Math.floor(i * bucketSize) + 1;
    const end = Math.min(Math.floor((i + 1) * bucketSize) + 1, n);
    const a = points[anchor];
    let maxArea = -1;
    let pick = start;
    for (let j = start; j < end; j++) {
      const area = Math.abs((a.x - avgX) * (points[j].y - a.y) - (a.x - points[j].x) * (avgY - a.y));
      if (area > maxArea) {
        maxArea = area;
        pick = j;
      }
    }
    out.push(points[pick]);
    anchor = pick;
  }

  out.push(points[n - 1]);
  return out;
}
