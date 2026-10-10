/** Log feed: the history of a text signal as timestamped lines, newest first. Pure. */
import type { SignalValue } from '../../data/types';

export type LogLine = { timestampMicros: number; text: string };

/**
 * Records each change of value (repeats are collapsed) and keeps the newest `capacity` lines.
 * Empty strings are not logged. Lines with a timestamp at or before `clearedAt` are hidden.
 */
export function logLines(
  samples: readonly { timestampMicros: number; value: SignalValue }[],
  capacity: number,
  clearedAt: number | null = null
): LogLine[] {
  const lines: LogLine[] = [];
  let previous: string | null = null;
  for (const sample of samples) {
    if (typeof sample.value !== 'string') continue;
    if (sample.value === previous) continue;
    previous = sample.value;
    if (sample.value === '') continue;
    if (clearedAt !== null && sample.timestampMicros <= clearedAt) continue;
    lines.push({ timestampMicros: sample.timestampMicros, text: sample.value });
  }
  return lines.slice(-Math.max(1, capacity)).reverse();
}
