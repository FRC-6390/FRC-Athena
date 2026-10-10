/**
 * Angle and drawing geometry shared by the robot widgets. Pure functions, no DOM.
 *
 * Dial convention: 0 degrees points up and angles increase clockwise (compass style).
 * Arm convention: 0 degrees points along +x and angles increase counter-clockwise (maths style).
 */

export type AngleUnit = 'deg' | 'rad';

export type Point = { x: number; y: number };

export const RAD_TO_DEG = 180 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180;

export function toDegrees(value: number, unit: AngleUnit): number {
  return unit === 'rad' ? value * RAD_TO_DEG : value;
}

export function toRadians(value: number, unit: AngleUnit): number {
  return unit === 'rad' ? value : value * DEG_TO_RAD;
}

/** Wraps degrees into (-180, 180]. */
export function wrapDegrees(deg: number): number {
  if (!Number.isFinite(deg)) return 0;
  const wrapped = ((((deg + 180) % 360) + 360) % 360) - 180;
  return wrapped === -180 ? 180 : wrapped;
}

/** Wraps degrees into [0, 360). */
export function normalizeDegrees(deg: number): number {
  if (!Number.isFinite(deg)) return 0;
  return ((deg % 360) + 360) % 360;
}

/** Signed shortest rotation from `from` to `to`, in degrees within (-180, 180]. */
export function shortestDelta(from: number, to: number): number {
  return wrapDegrees(to - from);
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/** Maps `value` from [inMin, inMax] onto [outMin, outMax]; a degenerate input range maps to outMin. */
export function mapRange(value: number, inMin: number, inMax: number, outMin: number, outMax: number): number {
  const span = inMax - inMin;
  if (span === 0 || !Number.isFinite(span)) return outMin;
  return outMin + ((value - inMin) / span) * (outMax - outMin);
}

/** Point on a compass-style circle: 0 degrees is up, clockwise positive. */
export function compassPoint(cx: number, cy: number, radius: number, deg: number): Point {
  const rad = deg * DEG_TO_RAD;
  return { x: cx + radius * Math.sin(rad), y: cy - radius * Math.cos(rad) };
}

/**
 * SVG path for a clockwise arc from `startDeg` to `endDeg` on a compass circle. Spans of 360 degrees or
 * more are clamped just under a full ring; zero or negative spans give an empty string.
 */
export function compassArcPath(cx: number, cy: number, radius: number, startDeg: number, endDeg: number): string {
  let span = endDeg - startDeg;
  if (!Number.isFinite(span) || span <= 0 || radius <= 0) return '';
  if (span >= 360) span = 359.999;
  const start = compassPoint(cx, cy, radius, startDeg);
  const end = compassPoint(cx, cy, radius, startDeg + span);
  const largeArc = span > 180 ? 1 : 0;
  return `M ${round(start.x)} ${round(start.y)} A ${round(radius)} ${round(radius)} 0 ${largeArc} 1 ${round(end.x)} ${round(end.y)}`;
}

/** Two-link planar forward kinematics. Joint angles are maths-convention; the elbow is relative to the upper link. */
export function twoLinkJoints(
  origin: Point,
  shoulderDeg: number,
  elbowDeg: number,
  upper: number,
  lower: number,
  scale: number
): { elbow: Point; tip: Point } {
  const a1 = shoulderDeg * DEG_TO_RAD;
  const a2 = (shoulderDeg + elbowDeg) * DEG_TO_RAD;
  const elbow = { x: origin.x + upper * scale * Math.cos(a1), y: origin.y - upper * scale * Math.sin(a1) };
  const tip = { x: elbow.x + lower * scale * Math.cos(a2), y: elbow.y - lower * scale * Math.sin(a2) };
  return { elbow, tip };
}

/** Pixels per unit so that a planar reach of `reach` units (radius) fits a box with `pad` px margins. */
export function fitScale(reach: number, width: number, height: number, pad: number): number {
  const usableW = Math.max(1, width - pad * 2);
  const usableH = Math.max(1, height - pad * 2);
  if (!(reach > 0)) return 1;
  return Math.min(usableW, usableH) / (2 * reach);
}

/** Rounds to two decimals for compact SVG attributes. */
export function round(n: number): number {
  return Math.round(n * 100) / 100;
}
