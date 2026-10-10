/**
 * Pure numeric helpers for the Controls pack: safe ranges, step maths and the "confirm large change" guard.
 * Widgets never block a write on lock state; these helpers only decide when a human must confirm first.
 */

export type NumericRange = { min: number; max: number };

/** A finite number from a setting, or null. Blank number settings are stored as NaN or null. */
export function finiteSetting(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}

/**
 * Range for steppers and sliders. User bounds win. Otherwise the range is a window around the current robot
 * value (0 to max(1, |v| * 4), mirrored when the value is negative), so a stray drag cannot jump to the far end.
 */
export function safeRange(current: unknown, userMin?: unknown, userMax?: unknown): NumericRange {
  const v = finiteSetting(current) ?? 0;
  const span = Math.max(1, Math.abs(v) * 4);
  const lo = finiteSetting(userMin);
  const hi = finiteSetting(userMax);
  const min = lo ?? (v < 0 ? -span : 0);
  const max = hi ?? span;
  return min <= max ? { min, max } : { min: max, max: min };
}

export function clamp(value: number, range?: NumericRange): number {
  if (!range) return value;
  return Math.min(range.max, Math.max(range.min, value));
}

/** Decimal places implied by a step, capped at 6 (0.25 gives 2, 1e-7 gives 6). */
export function decimalsOf(step: number): number {
  if (!Number.isFinite(step) || step <= 0) return 0;
  const text = step.toString();
  const exp = text.indexOf('e-');
  if (exp >= 0) return Math.min(6, parseInt(text.slice(exp + 2), 10));
  const dot = text.indexOf('.');
  return dot < 0 ? 0 : Math.min(6, text.length - dot - 1);
}

/** `value + direction * step`, rounded to the step's precision, then clamped to `range`. */
export function nudge(value: number, direction: 1 | -1, step: number, range?: NumericRange): number {
  const safeStep = Number.isFinite(step) && step > 0 ? step : 1;
  const next = Number((value + direction * safeStep).toFixed(decimalsOf(safeStep)));
  return clamp(next, range);
}

/** Parses typed text. Null for blank, non-numeric or non-finite input. */
export function parseNumberInput(text: string | number | null | undefined): number | null {
  if (text === null || text === undefined) return null;
  const trimmed = String(text).trim();
  if (trimmed === '') return null;
  const n = Number(trimmed);
  return Number.isFinite(n) ? n : null;
}

export type ConfirmReason = 'range' | 'change' | 'unknown';

/**
 * Why a write needs an inline confirmation, or null when it can go straight out.
 * - `range`: outside the user's own min/max.
 * - `change`: more than 50% away from the robot's current value (any non-zero value from a zero start).
 * - `unknown`: the robot's current value is not known, so the size of the jump cannot be judged.
 */
export function confirmReason(
  current: unknown,
  next: number,
  bounds: { userMin?: unknown; userMax?: unknown } = {}
): ConfirmReason | null {
  const lo = finiteSetting(bounds.userMin);
  const hi = finiteSetting(bounds.userMax);
  if ((lo !== null && next < lo) || (hi !== null && next > hi)) return 'range';
  const cur = finiteSetting(current);
  if (cur === null) return 'unknown';
  if (next === cur) return null;
  if (cur === 0) return 'change';
  return Math.abs(next - cur) > 0.5 * Math.abs(cur) ? 'change' : null;
}

/** Human summary of a change, e.g. `12 to 30 (+150%)`. */
export function describeChange(current: unknown, next: number): string {
  const cur = finiteSetting(current);
  if (cur === null) return `to ${next}`;
  if (cur === 0) return `from 0 to ${next}`;
  const pct = Math.round(((next - cur) / Math.abs(cur)) * 100);
  return `${cur} to ${next} (${pct >= 0 ? '+' : ''}${pct}%)`;
}
