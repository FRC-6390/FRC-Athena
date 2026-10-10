/**
 * Pure helpers for the PID tuner: which signals look like gains, and where a sibling Restore action lives.
 */
import type { SignalDescriptor } from '../../data/types';

const GAIN_NAMES = new Set(['p', 'i', 'd', 'kp', 'ki', 'kd', 'ks', 'kv', 'ka', 'kg', 'izone']);

/** True when the signal's last path segment is a conventional gain name (case-insensitive). */
export function isGainName(name: string): boolean {
  return GAIN_NAMES.has(name.toLowerCase());
}

/** Gain-like writable numbers rank above generic numbers, and well above the plain number input. */
export function gainScore(signal: SignalDescriptor): number {
  if (!signal.writable || (signal.type !== 'f64' && signal.type !== 'i64')) return 0;
  return isGainName(signal.name) ? 85 : 20;
}

function parentPath(path: string): string | null {
  const cut = path.lastIndexOf('/');
  return cut > 0 ? path.slice(0, cut) : null;
}

/**
 * The Restore action that belongs with these gains: `<dir>/Restore` or `<dir>/Actions/Restore`, checked in the
 * gains' own folder and its parent (so `Config/PID/p` finds `Config/Restore`).
 */
export function findRestoreAction(
  gains: readonly SignalDescriptor[],
  signals: readonly SignalDescriptor[]
): SignalDescriptor | undefined {
  const first = gains[0];
  if (!first) return undefined;
  const candidates = new Set<string>();
  let dir = parentPath(first.path);
  for (let depth = 0; depth < 2 && dir; depth++) {
    candidates.add(`${dir}/Restore`);
    candidates.add(`${dir}/Actions/Restore`);
    dir = parentPath(dir);
  }
  const isAction = (s: SignalDescriptor) => s.role === 'action' || s.localPath.startsWith('Actions/');
  // Prefer the closest folder: the earliest candidate in the list wins.
  const ordered = [...candidates];
  for (const path of ordered) {
    const hit = signals.find((s) => s.path === path && isAction(s));
    if (hit) return hit;
  }
  return undefined;
}
