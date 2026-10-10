import { isLayoutWidgetKind, type DashboardWidget } from '../dashboard';
import type { SignalRow } from '../arcp';

/**
 * Resolves the signal a classic widget shows. A widget may be bound by id (`signalId`), by a topic or
 * signal path in its config, or by ids/paths nested in its config (graph series, controller params, ...).
 * Resolution is tolerant of path spelling (slashes, case, punctuation) and prefers a live value over a
 * placeholder one.
 */
export type SignalResolver = {
  resolveWidgetSignal: (widget: DashboardWidget) => SignalRow | null;
  unavailableSignalHint: (widget: DashboardWidget) => string;
};

export function normalizeSignalPathForLookup(rawPath: string): string {
  return rawPath
    .trim()
    .replace(/\\/g, '/')
    .replace(/\/+/g, '/')
    .replace(/^\/+|\/+$/g, '')
    .toLowerCase();
}

export function canonicalSignalToken(value: string): string {
  return value.toLowerCase().replace(/[^a-z0-9]/g, '');
}

export function canonicalSignalPathForLookup(rawPath: string): string {
  return rawPath
    .trim()
    .replace(/\\/g, '/')
    .replace(/\/+/g, '/')
    .replace(/^\/+|\/+$/g, '')
    .split('/')
    .map((segment) => canonicalSignalToken(segment))
    .filter((segment) => segment.length > 0)
    .join('/');
}

export function isPlaceholderSignalValue(row: SignalRow): boolean {
  const value = row.value.trim();
  return value.length === 0 || value === '-';
}

const NESTED_SIGNAL_KEYS = new Set(['series', 'actions', 'columns', 'signals', 'targets']);

/** Signal ids referenced from a widget config (`*signalId` keys, and ids inside nested signal lists). */
export function collectConfiguredSignalIds(value: unknown, out: Set<number>): void {
  if (value === null || value === undefined) return;
  if (typeof value === 'number') {
    if (Number.isFinite(value) && value > 0) out.add(Math.floor(value));
    return;
  }
  if (Array.isArray(value)) {
    for (const item of value) collectConfiguredSignalIds(item, out);
    return;
  }
  if (typeof value !== 'object') return;
  for (const [key, nested] of Object.entries(value as Record<string, unknown>)) {
    if (key.toLowerCase().endsWith('signalid')) {
      collectConfiguredSignalIds(nested, out);
      continue;
    }
    if (NESTED_SIGNAL_KEYS.has(key)) collectConfiguredSignalIds(nested, out);
  }
}

/** Signal paths referenced from a widget config (`*signalPath` keys, and paths inside nested lists). */
export function collectConfiguredSignalPaths(value: unknown, out: Set<string>): void {
  if (value === null || value === undefined) return;
  if (typeof value === 'string') {
    const trimmed = value.trim();
    if (trimmed) out.add(trimmed);
    return;
  }
  if (Array.isArray(value)) {
    for (const item of value) collectConfiguredSignalPaths(item, out);
    return;
  }
  if (typeof value !== 'object') return;
  for (const [key, nested] of Object.entries(value as Record<string, unknown>)) {
    if (key.toLowerCase().endsWith('signalpath') || key === 'signalPath') {
      collectConfiguredSignalPaths(nested, out);
      continue;
    }
    if (NESTED_SIGNAL_KEYS.has(key)) collectConfiguredSignalPaths(nested, out);
  }
}

/** Builds the resolver for one snapshot of the signal table. */
export function createSignalResolver(
  signals: readonly SignalRow[],
  signalById: ReadonlyMap<number, SignalRow>
): SignalResolver {
  const signalByNormalizedPath = new Map<string, SignalRow>();
  for (const entry of signals) {
    const normalized = normalizeSignalPathForLookup(entry.path);
    if (!normalized || signalByNormalizedPath.has(normalized)) continue;
    signalByNormalizedPath.set(normalized, entry);
  }

  function resolveSignalFromCanonicalTopic(topicPath: string, preferredLeafPath = ''): SignalRow | null {
    const canonicalTopic = canonicalSignalPathForLookup(topicPath);
    if (!canonicalTopic) return null;
    const preferredLeaf = canonicalSignalToken(preferredLeafPath.slice(preferredLeafPath.lastIndexOf('/') + 1));

    const candidates: SignalRow[] = [];
    for (const entry of signals) {
      const canonicalPath = canonicalSignalPathForLookup(entry.path);
      if (!canonicalPath) continue;
      if (canonicalPath === canonicalTopic || canonicalPath.startsWith(`${canonicalTopic}/`)) {
        candidates.push(entry);
      }
    }
    if (candidates.length === 0) return null;

    const scored = candidates
      .map((entry) => {
        const canonicalLeaf = canonicalSignalToken(entry.path.slice(entry.path.lastIndexOf('/') + 1));
        let score = 0;
        if (preferredLeaf && canonicalLeaf === preferredLeaf) score += 8;
        if (!isPlaceholderSignalValue(entry)) score += 4;
        if (entry.access === 'observe') score += 1;
        return { entry, score };
      })
      .sort((a, b) => b.score - a.score || a.entry.path.localeCompare(b.entry.path));

    return scored[0]?.entry ?? null;
  }

  function resolveSignalFromPath(rawPath: string, topicPath: string): SignalRow | null {
    const cleanedRaw = rawPath.trim();
    if (!cleanedRaw) return null;

    const normalizedTopic = normalizeSignalPathForLookup(topicPath);
    let candidate = cleanedRaw;
    if (candidate.startsWith('./')) {
      candidate = candidate.slice(2);
    }

    let resolvedPath = '';
    if (candidate.startsWith('/')) {
      resolvedPath = normalizeSignalPathForLookup(candidate.slice(1));
    } else {
      const normalizedCandidate = normalizeSignalPathForLookup(candidate);
      if (!normalizedCandidate) return null;

      const hasQualifiedRoot = normalizedCandidate.startsWith('athena/') || normalizedCandidate.startsWith('robot/');
      resolvedPath =
        hasQualifiedRoot || !normalizedTopic
          ? normalizedCandidate
          : normalizeSignalPathForLookup(`${normalizedTopic}/${normalizedCandidate}`);
    }

    if (!resolvedPath) return null;
    const exact = signalByNormalizedPath.get(resolvedPath);
    if (exact) return exact;

    const canonicalResolved = canonicalSignalPathForLookup(resolvedPath);
    if (canonicalResolved) {
      for (const entry of signals) {
        if (canonicalSignalPathForLookup(entry.path) === canonicalResolved) return entry;
      }
    }

    if (!candidate.includes('/')) {
      const normalizedLeaf = normalizeSignalPathForLookup(candidate);
      if (!normalizedLeaf) return null;
      const canonicalTopic = canonicalSignalPathForLookup(normalizedTopic);
      for (const entry of signals) {
        const normalizedPath = normalizeSignalPathForLookup(entry.path);
        if (!normalizedPath) continue;
        if (normalizedTopic) {
          const inNormalizedScope = normalizedPath === normalizedTopic || normalizedPath.startsWith(`${normalizedTopic}/`);
          const canonicalPath = canonicalSignalPathForLookup(entry.path);
          const inCanonicalScope =
            canonicalTopic.length > 0 && (canonicalPath === canonicalTopic || canonicalPath.startsWith(`${canonicalTopic}/`));
          if (!inNormalizedScope && !inCanonicalScope) continue;
        }
        const leaf = normalizedPath.slice(normalizedPath.lastIndexOf('/') + 1);
        if (leaf === normalizedLeaf || leaf.endsWith(normalizedLeaf)) return entry;
      }
    }

    return null;
  }

  function resolveWidgetSignal(widget: DashboardWidget): SignalRow | null {
    if (isLayoutWidgetKind(widget.kind)) return null;

    const directSignal =
      Number.isFinite(widget.signalId) && widget.signalId > 0 ? signalById.get(widget.signalId) ?? null : null;
    const config = widget.config;
    if (!config || typeof config !== 'object' || Array.isArray(config)) {
      return directSignal;
    }

    const topicPath = typeof config.topicPath === 'string' ? config.topicPath.trim() : '';
    const normalizedTopic = normalizeSignalPathForLookup(topicPath);

    const configuredSignalPath = typeof config.signalPath === 'string' ? config.signalPath.trim() : '';
    if (configuredSignalPath) {
      const resolved = resolveSignalFromPath(configuredSignalPath, topicPath);
      if (resolved) return resolved;
    }

    if (directSignal) {
      if (!normalizedTopic) return directSignal;
      const normalizedDirect = normalizeSignalPathForLookup(directSignal.path);
      if (normalizedDirect === normalizedTopic || normalizedDirect.startsWith(`${normalizedTopic}/`)) {
        if (!isPlaceholderSignalValue(directSignal)) return directSignal;
        const canonicalFallback = resolveSignalFromCanonicalTopic(topicPath, directSignal.path);
        if (canonicalFallback && canonicalFallback.signal_id !== directSignal.signal_id) return canonicalFallback;
        return directSignal;
      }
    }

    const candidateIds = new Set<number>();
    collectConfiguredSignalIds(config, candidateIds);
    let bestIdCandidate: SignalRow | null = null;
    for (const id of candidateIds) {
      const resolved = signalById.get(id);
      if (!resolved) continue;
      if (!bestIdCandidate || !isPlaceholderSignalValue(resolved)) {
        bestIdCandidate = resolved;
      }
      if (!isPlaceholderSignalValue(resolved)) break;
    }
    if (bestIdCandidate) return bestIdCandidate;

    const candidatePaths = new Set<string>();
    collectConfiguredSignalPaths(config, candidatePaths);
    for (const rawPath of candidatePaths) {
      const resolved = resolveSignalFromPath(rawPath, topicPath);
      if (resolved) return resolved;
    }

    if (normalizedTopic) {
      const exact = signalByNormalizedPath.get(normalizedTopic);
      if (exact && !isPlaceholderSignalValue(exact)) return exact;
      for (const entry of signals) {
        const normalizedPath = normalizeSignalPathForLookup(entry.path);
        if (normalizedPath.startsWith(`${normalizedTopic}/`)) {
          if (!isPlaceholderSignalValue(entry)) return entry;
          if (!exact) return entry;
        }
      }
      const canonicalFallback = resolveSignalFromCanonicalTopic(topicPath, directSignal?.path ?? '');
      if (canonicalFallback) return canonicalFallback;
      if (exact) return exact;
    }

    if (directSignal) return directSignal;
    return null;
  }

  function unavailableSignalHint(widget: DashboardWidget): string {
    if (widget.signalId > 0) return `signal #${widget.signalId}`;
    const config = widget.config;
    if (!config || typeof config !== 'object' || Array.isArray(config)) return 'no bound signal';
    const topicPath = typeof config.topicPath === 'string' ? config.topicPath.trim() : '';
    if (topicPath) return topicPath;
    const signalPath = typeof config.signalPath === 'string' ? config.signalPath.trim() : '';
    if (signalPath) return signalPath;
    return 'no bound signal';
  }

  return { resolveWidgetSignal, unavailableSignalHint };
}
