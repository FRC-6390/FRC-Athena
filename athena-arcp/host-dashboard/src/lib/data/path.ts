import type { SignalDescriptor } from './types';

/** Top-level mechanism groups (telemetry-and-simulation.md, "Mechanism-scoped telemetry"). */
export const MECHANISM_GROUPS = [
  'State',
  'Values',
  'Actions',
  'Devices',
  'Controls',
  'Hooks',
  'Trace'
] as const;

export type MechanismGroup = (typeof MECHANISM_GROUPS)[number];

const TELEMETRY_PREFIX = '/Athena/Mechanisms/';

/** Normalises a telemetry path to the mechanism-tree form `robot/arm/State/x`. */
export function normalizePath(path: string): string {
  let out = path.startsWith(TELEMETRY_PREFIX) ? path.slice(TELEMETRY_PREFIX.length) : path;
  out = out.replace(/^\/+/, '').replace(/\/+$/, '');
  return out;
}

export type ParsedPath = {
  /** Mechanism path, e.g. `robot/turret`. Empty for a root-level signal with no group. */
  mechanismPath: string;
  /** Group segment if the path contains one (`State`, `Controls`, ...). */
  group: MechanismGroup | null;
  /** Remaining segments after the group (or after the mechanism when no group is present). */
  localPath: string;
  name: string;
};

/**
 * Splits a dictionary path into mechanism, group and local parts. The mechanism path is every segment
 * before the first group segment, so nested child mechanisms (`robot/arm/shoulder/Values/x`) stay nested.
 */
export function parsePath(rawPath: string): ParsedPath {
  const segments = normalizePath(rawPath).split('/').filter((s) => s.length > 0);
  const groupIndex = segments.findIndex((s) =>
    (MECHANISM_GROUPS as readonly string[]).includes(s)
  );
  const name = segments.length > 0 ? segments[segments.length - 1] : '';
  if (groupIndex < 0) {
    return {
      mechanismPath: segments.slice(0, -1).join('/'),
      group: null,
      localPath: name,
      name
    };
  }
  return {
    mechanismPath: segments.slice(0, groupIndex).join('/'),
    group: segments[groupIndex] as MechanismGroup,
    localPath: segments.slice(groupIndex).join('/'),
    name
  };
}

/** Builds the derived descriptor fields for a signal path. */
export function describePath(path: string): Pick<
  SignalDescriptor,
  'path' | 'mechanismPath' | 'localPath' | 'name'
> {
  const normalized = normalizePath(path);
  const parsed = parsePath(normalized);
  return {
    path: normalized,
    mechanismPath: parsed.mechanismPath,
    localPath: parsed.localPath,
    name: parsed.name
  };
}
