import { SignalRow } from '../arcp';
import { leafPath } from '../dashboard';

export function isObject(value: unknown): value is Record<string, unknown> {
  return !!value && typeof value === 'object' && !Array.isArray(value);
}

export function toNumber(value: unknown): number | null {
  if (typeof value === 'number') {
    return Number.isFinite(value) ? value : null;
  }

  if (typeof value === 'string') {
    const numeric = Number(value.trim());
    return Number.isFinite(numeric) ? numeric : null;
  }

  return null;
}

export function toBoolean(value: unknown, fallback: boolean): boolean {
  if (typeof value === 'boolean') return value;
  return fallback;
}

export function toString(value: unknown, fallback = ''): string {
  if (typeof value === 'string') return value;
  return fallback;
}

export function looksLikeColor(value: string): boolean {
  const raw = value.trim();
  if (!raw) return false;
  if (/^#([0-9a-fA-F]{3}|[0-9a-fA-F]{6}|[0-9a-fA-F]{8})$/.test(raw)) return true;
  if (/^(rgb|rgba|hsl|hsla)\(/i.test(raw)) return true;
  if (/^[a-zA-Z]+$/.test(raw)) return true;
  return false;
}

export function clamp01(value: number): number {
  return Math.max(0, Math.min(1, value));
}

export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

export function normalizeSignalId(value: unknown): number | null {
  const parsed = toNumber(value);
  if (parsed === null || parsed <= 0) return null;
  return Math.floor(parsed);
}

export function hasOwn(obj: Record<string, unknown>, key: string): boolean {
  return Object.prototype.hasOwnProperty.call(obj, key);
}

export function isExplicitSignalDisable(value: unknown): boolean {
  return value === null || (typeof value === 'string' && value.trim().length === 0);
}

export function resolveSignalIdFromKeys(
  base: Record<string, unknown>,
  keys: readonly string[],
  defaultSignalId: number | null,
  signals: SignalRow[]
): number | null {
  for (const key of keys) {
    if (!hasOwn(base, key)) continue;
    const raw = base[key];
    if (isExplicitSignalDisable(raw)) return null;
    const parsed = normalizeSignalId(raw);
    if (parsed !== null && signals.some((signal) => signal.signal_id === parsed)) {
      return parsed;
    }
    // Stale/missing signal ids in persisted layouts should fall back to current auto-binding.
    continue;
  }

  if (defaultSignalId !== null && signals.some((signal) => signal.signal_id === defaultSignalId)) {
    return defaultSignalId;
  }
  return null;
}

export function normalizeSignalPath(rawPath: string): string {
  const trimmed = rawPath.replace(/\\/g, '/').trim();
  if (!trimmed) return '';
  const rawSegments = trimmed.split('/');
  const segments: string[] = [];
  for (const rawSegment of rawSegments) {
    const segment = rawSegment.trim();
    if (!segment || segment === '.') continue;
    if (segment === '..') {
      segments.pop();
      continue;
    }
    segments.push(segment);
  }
  return segments.join('/');
}

export function canonicalSignalToken(value: string): string {
  return value.toLowerCase().replace(/[^a-z0-9]/g, '');
}

export function canonicalSignalPath(rawPath: string): string {
  const normalized = normalizeSignalPath(rawPath);
  if (!normalized) return '';
  return normalized
    .split('/')
    .map((segment) => canonicalSignalToken(segment))
    .filter((segment) => segment.length > 0)
    .join('/');
}

export function isPathWithinScope(path: string, scope: string): boolean {
  const normalizedPath = normalizeSignalPath(path);
  const normalizedScope = normalizeSignalPath(scope);
  if (!normalizedScope) return true;
  if (
    normalizedPath === normalizedScope ||
    normalizedPath.startsWith(`${normalizedScope}/`)
  ) {
    return true;
  }
  const canonicalPath = canonicalSignalPath(normalizedPath);
  const canonicalScope = canonicalSignalPath(normalizedScope);
  if (!canonicalScope) return true;
  return (
    canonicalPath === canonicalScope ||
    canonicalPath.startsWith(`${canonicalScope}/`)
  );
}

export function scopedSignals(signals: SignalRow[], anchorParent: string): SignalRow[] {
  const normalizedAnchorParent = normalizeSignalPath(anchorParent);
  if (!normalizedAnchorParent) return signals;
  const scoped = signals.filter((signal) =>
    isPathWithinScope(parentPath(signal.path), normalizedAnchorParent)
  );
  return scoped.length > 0 ? scoped : signals;
}

export function resolveSignalIdFromPath(
  rawPath: string,
  topicPath: string,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): number | null {
  const cleanedRaw = rawPath.trim();
  if (!cleanedRaw) return null;

  const normalizedTopic = normalizeSignalPath(topicPath);
  const fallbackParent = parentPath(fallbackSignal.path);

  let candidate = cleanedRaw;
  if (candidate.startsWith('./')) {
    candidate = candidate.slice(2);
  }

  let resolvedPath = '';
  if (candidate.startsWith('/')) {
    resolvedPath = normalizeSignalPath(candidate.slice(1));
  } else {
    const normalizedCandidate = normalizeSignalPath(candidate);
    if (!normalizedCandidate) return null;

    const hasQualifiedRoot =
      normalizedCandidate.startsWith('Athena/') ||
      normalizedCandidate.startsWith('athena/') ||
      normalizedCandidate.startsWith('Robot/') ||
      normalizedCandidate.startsWith('robot/');

    if (hasQualifiedRoot) {
      resolvedPath = normalizedCandidate;
    } else {
      const base = normalizedTopic || fallbackParent;
      resolvedPath = base
        ? normalizeSignalPath(`${base}/${normalizedCandidate}`)
        : normalizedCandidate;
    }
  }

  if (!resolvedPath) return null;
  const normalizedResolved = resolvedPath.toLowerCase();
  const exact = signals.find(
    (signal) => normalizeSignalPath(signal.path).toLowerCase() === normalizedResolved
  );
  if (exact) return exact.signal_id;

  const canonicalResolved = canonicalSignalPath(resolvedPath);
  if (canonicalResolved) {
    const canonicalMatch = signals.find(
      (signal) => canonicalSignalPath(signal.path) === canonicalResolved
    );
    if (canonicalMatch) return canonicalMatch.signal_id;
  }

  if (!candidate.includes('/')) {
    const scopedParent = normalizedTopic || fallbackParent;
    const byLeaf = findSignalByLeaf(signals, scopedParent, [candidate]);
    if (byLeaf) return byLeaf.signal_id;
  }

  return null;
}

export function resolveConfiguredSignalId(
  base: Record<string, unknown>,
  idKey: string,
  pathKey: string,
  defaultSignalId: number | null,
  topicPath: string,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): number | null {
  if (hasOwn(base, idKey) && isExplicitSignalDisable(base[idKey])) {
    return null;
  }

  const rawPath = toString(base[pathKey], '').trim();
  if (rawPath) {
    const resolved = resolveSignalIdFromPath(rawPath, topicPath, fallbackSignal, signals);
    if (resolved !== null) return resolved;
  }

  if (hasOwn(base, idKey)) {
    const direct = normalizeSignalId(base[idKey]);
    if (direct !== null && signals.some((signal) => signal.signal_id === direct)) {
      return direct;
    }
    return null;
  }

  if (defaultSignalId !== null && signals.some((signal) => signal.signal_id === defaultSignalId)) {
    return defaultSignalId;
  }
  return null;
}

export function looksLikeUrl(value: string): boolean {
  const raw = value.trim().toLowerCase();
  if (!raw) return false;
  return (
    raw.startsWith('http://') ||
    raw.startsWith('https://') ||
    raw.startsWith('rtsp://') ||
    raw.startsWith('mjpeg://') ||
    raw.startsWith('data:image/') ||
    raw.startsWith('/')
  );
}

export function normalizeToken(raw: string): string {
  return raw.toLowerCase().replace(/[^a-z0-9]/g, '');
}

export function parentPath(path: string): string {
  const parts = path.split('/').filter(Boolean);
  if (parts.length <= 1) return '';
  return parts.slice(0, -1).join('/');
}

export function isNumericSignal(signal: SignalRow): boolean {
  return signal.signal_type === 'f64' || signal.signal_type === 'i64';
}

export function signalRole(signal: SignalRow): string {
  if (signal.access === 'invoke' || signal.kind === 'command') return 'action';
  if (signal.access === 'write') return 'tunable';
  if (
    signal.signal_type === 'bool' &&
    signal.policy === 'on_change' &&
    signal.durability !== 'volatile'
  ) {
    return 'state';
  }
  return 'metric';
}

export function isNtPublishToggleSignal(signal: SignalRow): boolean {
  const token = normalizeToken(signal.path);
  return token.includes('publish');
}

export function findSignalByLeaf(
  signals: SignalRow[],
  anchorParent: string,
  candidates: string[],
  allowedTypes?: string[]
): SignalRow | null {
  const normalizedCandidates = new Set(candidates.map((candidate) => normalizeToken(candidate)));
  const fullPool = scopedSignals(signals, anchorParent);
  const isPlaceholder = (signal: SignalRow): boolean => {
    const value = signal.value.trim();
    return value.length === 0 || value === '-';
  };

  const matchesCandidate = (signal: SignalRow): boolean => {
    const token = normalizeToken(leafPath(signal.path));
    if (!token) return false;
    return normalizedCandidates.has(token);
  };

  const matchesSuffix = (signal: SignalRow): boolean => {
    const token = normalizeToken(leafPath(signal.path));
    if (!token) return false;
    for (const candidate of normalizedCandidates) {
      if (token.endsWith(candidate)) {
        return true;
      }
    }
    return false;
  };

  for (const signal of fullPool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (allowedTypes && !allowedTypes.includes(signal.signal_type)) continue;
    if (!matchesCandidate(signal)) continue;
    if (isPlaceholder(signal)) continue;
    return signal;
  }

  for (const signal of fullPool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (allowedTypes && !allowedTypes.includes(signal.signal_type)) continue;
    if (!matchesCandidate(signal)) continue;
    return signal;
  }

  for (const signal of fullPool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (allowedTypes && !allowedTypes.includes(signal.signal_type)) continue;
    if (!matchesSuffix(signal)) continue;
    if (isPlaceholder(signal)) continue;
    return signal;
  }

  for (const signal of fullPool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (allowedTypes && !allowedTypes.includes(signal.signal_type)) continue;
    if (!matchesSuffix(signal)) continue;
    return signal;
  }

  return null;
}

export function findWritableNumericSignalByLeaf(
  signals: SignalRow[],
  anchorParent: string,
  candidates: string[]
): SignalRow | null {
  const pool = scopedSignals(signals, anchorParent);
  const normalizedCandidates = candidates.map((candidate) => normalizeToken(candidate));
  const isPlaceholder = (signal: SignalRow): boolean => {
    const value = signal.value.trim();
    return value.length === 0 || value === '-';
  };

  for (const signal of pool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (!isNumericSignal(signal) || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.includes(token) && !isPlaceholder(signal)) {
      return signal;
    }
  }

  for (const signal of pool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (!isNumericSignal(signal) || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.includes(token)) {
      return signal;
    }
  }

  for (const signal of pool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (!isNumericSignal(signal) || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.some((candidate) => token.endsWith(candidate) || token.includes(candidate))) {
      if (isPlaceholder(signal)) continue;
      return signal;
    }
  }

  for (const signal of pool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (!isNumericSignal(signal) || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.some((candidate) => token.endsWith(candidate) || token.includes(candidate))) {
      return signal;
    }
  }

  return null;
}

export function parseNumericArray(raw: string): number[] {
  const trimmed = raw.trim();
  if (!trimmed) return [];

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    const arrayLike = Array.isArray(parsed)
      ? parsed
      : isObject(parsed) && Array.isArray(parsed.values)
        ? parsed.values
        : null;
    if (arrayLike) {
      return arrayLike
        .map((entry) => toNumber(entry))
        .filter((entry): entry is number => entry !== null)
        .slice(0, 4096);
    }
  } catch {
    // fall through to token extraction
  }

  const matches = trimmed.match(/[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?/g);
  if (!matches) return [];
  return matches
    .map((entry) => Number(entry))
    .filter((entry) => Number.isFinite(entry))
    .slice(0, 4096);
}
