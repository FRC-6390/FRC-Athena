/** Alliance and driver-station parsing for the alliance widget and the field perspective. */

export type Alliance = 'red' | 'blue';

/**
 * Reads an alliance from a string ("Red", "RED ALLIANCE", "blue") or a boolean. Booleans map through
 * `trueMeans`, so a robot that publishes `isRed` is configured with `trueMeans = 'red'`.
 */
export function parseAlliance(value: unknown, trueMeans: Alliance = 'red'): Alliance | null {
  if (typeof value === 'boolean') {
    if (value) return trueMeans;
    return trueMeans === 'red' ? 'blue' : 'red';
  }
  if (typeof value !== 'string') return null;
  const text = value.trim().toLowerCase();
  if (/\bred\b/.test(text)) return 'red';
  if (/\bblue\b/.test(text)) return 'blue';
  return null;
}

/** Driver station number 1 to 3 (anything else is treated as unknown). */
export function parseStation(value: unknown): number | null {
  const n = typeof value === 'bigint' ? Number(value) : value;
  if (typeof n !== 'number' || !Number.isInteger(n) || n < 1 || n > 3) return null;
  return n;
}

export function allianceLabel(alliance: Alliance | null): string {
  if (alliance === 'red') return 'Red';
  if (alliance === 'blue') return 'Blue';
  return 'Unknown';
}
