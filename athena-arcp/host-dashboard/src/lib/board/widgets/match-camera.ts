/** Camera stream helpers: URL validation and reconnect timing. Pure. */

/** Returns a problem description, or null when the URL can be used. */
export function streamUrlProblem(url: string): string | null {
  const trimmed = url.trim();
  if (trimmed === '') return 'No stream URL set';
  let parsed: URL;
  try {
    parsed = new URL(trimmed);
  } catch {
    return 'Stream URL is not valid';
  }
  if (parsed.protocol !== 'http:' && parsed.protocol !== 'https:') return 'Stream URL must be http or https';
  return null;
}

/** Backoff between reconnect attempts: 1 s, 2 s, 4 s, then 5 s. */
export function retryDelayMs(attempt: number): number {
  return Math.min(5000, 1000 * 2 ** Math.max(0, Math.min(attempt, 3)));
}

/** The stream URL with a cache-busting parameter after the first failure, so the browser reconnects. */
export function streamSource(url: string, attempt: number): string {
  const trimmed = url.trim();
  if (attempt === 0) return trimmed;
  return `${trimmed}${trimmed.includes('?') ? '&' : '?'}retry=${attempt}`;
}
