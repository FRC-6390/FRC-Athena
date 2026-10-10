/** Stopwatch and countdown state machine. Time is passed in, so the logic is deterministic under test. */

export type Stopwatch = {
  /** Time banked from earlier running periods. */
  bankedMs: number;
  /** Clock reading when the current running period began, or null when stopped. */
  startedAt: number | null;
};

export const STOPPED: Stopwatch = { bankedMs: 0, startedAt: null };

export function elapsedMs(sw: Stopwatch, now: number): number {
  return sw.bankedMs + (sw.startedAt === null ? 0 : Math.max(0, now - sw.startedAt));
}

export function start(sw: Stopwatch, now: number): Stopwatch {
  return sw.startedAt === null ? { bankedMs: sw.bankedMs, startedAt: now } : sw;
}

export function stop(sw: Stopwatch, now: number): Stopwatch {
  if (sw.startedAt === null) return sw;
  return { bankedMs: elapsedMs(sw, now), startedAt: null };
}

export function reset(): Stopwatch {
  return STOPPED;
}

/** Milliseconds left on a countdown, never below zero. */
export function remainingMs(targetSeconds: number, sw: Stopwatch, now: number): number {
  return Math.max(0, targetSeconds * 1000 - elapsedMs(sw, now));
}

/** m:ss.t (tenths). Negative input is clamped to zero. */
export function formatTenths(ms: number): string {
  const tenths = Math.floor(Math.max(0, ms) / 100);
  const minutes = Math.floor(tenths / 600);
  const seconds = Math.floor((tenths % 600) / 10);
  return `${minutes}:${String(seconds).padStart(2, '0')}.${tenths % 10}`;
}
