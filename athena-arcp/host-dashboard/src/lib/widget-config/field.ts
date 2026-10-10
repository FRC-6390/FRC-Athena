import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { clamp, findSignalByLeaf, isNumericSignal, isObject, normalizeToken, parentPath, resolveSignalIdFromKeys, toBoolean, toNumber, toString } from './common';

export const DEFAULT_FIELD_LENGTH_METERS = 16.54;

export const DEFAULT_FIELD_WIDTH_METERS = 8.02;

export type FieldWidgetConfig = {
  xSignalId: number | null;
  ySignalId: number | null;
  headingSignalId: number | null;
  trajectorySignalId: number | null;
  imageSignalId: number | null;
  imageUrl: string;
  imageOpacity: number;
  fieldLength: number;
  fieldWidth: number;
  allowPoseSet: boolean;
};

export type FieldPoint = {
  x: number;
  y: number;
};

export function defaultFieldSignalBinding(anchor: SignalRow, signals: SignalRow[]): FieldWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const xSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['x', 'xmeters', 'posex', 'robotx'],
    ['f64', 'i64']
  );
  const ySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['y', 'ymeters', 'posey', 'roboty'],
    ['f64', 'i64']
  );
  const headingSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['heading', 'headingdeg', 'yaw', 'yawdeg', 'theta', 'rotation', 'rotationdeg', 'angle', 'gyro'],
    ['f64', 'i64']
  );
  const trajectorySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['trajectory', 'traj', 'path'],
    ['string', 'string[]']
  );
  const imageSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['fieldimage', 'imageurl', 'image', 'fieldmap'],
    ['string', 'string[]']
  );

  const leaf = normalizeToken(leafPath(anchor.path));
  const xSignalId = xSignal?.signal_id ?? (isNumericSignal(anchor) && (leaf === 'x' || leaf.endsWith('x')) ? anchor.signal_id : null);
  const ySignalId = ySignal?.signal_id ?? (isNumericSignal(anchor) && (leaf === 'y' || leaf.endsWith('y')) ? anchor.signal_id : null);
  const headingSignalId = headingSignal?.signal_id ?? (isNumericSignal(anchor) && ['heading', 'yaw', 'theta', 'rotation', 'gyro'].some((token) => leaf.endsWith(token)) ? anchor.signal_id : null);

  const xWritable = xSignalId !== null && signals.find((signal) => signal.signal_id === xSignalId)?.access === 'write';
  const yWritable = ySignalId !== null && signals.find((signal) => signal.signal_id === ySignalId)?.access === 'write';

  return {
    xSignalId,
    ySignalId,
    headingSignalId,
    trajectorySignalId: trajectorySignal?.signal_id ?? null,
    imageSignalId: imageSignal?.signal_id ?? null,
    imageUrl: '/fields/FieldImage2026.svg',
    imageOpacity: 0.78,
    fieldLength: DEFAULT_FIELD_LENGTH_METERS,
    fieldWidth: DEFAULT_FIELD_WIDTH_METERS,
    allowPoseSet: !!xWritable && !!yWritable
  };
}

export function readFieldConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): FieldWidgetConfig {
  const defaults = defaultFieldSignalBinding(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    xSignalId: resolveSignalIdFromKeys(base, ['xSignalId'], defaults.xSignalId, signals),
    ySignalId: resolveSignalIdFromKeys(base, ['ySignalId'], defaults.ySignalId, signals),
    headingSignalId: resolveSignalIdFromKeys(
      base,
      ['headingSignalId'],
      defaults.headingSignalId,
      signals
    ),
    trajectorySignalId: resolveSignalIdFromKeys(
      base,
      ['trajectorySignalId'],
      defaults.trajectorySignalId,
      signals
    ),
    imageSignalId: resolveSignalIdFromKeys(base, ['imageSignalId'], defaults.imageSignalId, signals),
    imageUrl: toString(base.imageUrl, defaults.imageUrl),
    imageOpacity: clamp(toNumber(base.imageOpacity) ?? defaults.imageOpacity, 0, 1),
    fieldLength: Math.max(1, toNumber(base.fieldLength) ?? defaults.fieldLength),
    fieldWidth: Math.max(1, toNumber(base.fieldWidth) ?? defaults.fieldWidth),
    allowPoseSet: toBoolean(base.allowPoseSet, defaults.allowPoseSet)
  };
}

export function pointFromUnknown(value: unknown): FieldPoint | null {
  if (Array.isArray(value) && value.length >= 2) {
    const x = toNumber(value[0]);
    const y = toNumber(value[1]);
    if (x !== null && y !== null) {
      return { x, y };
    }
    return null;
  }

  if (!isObject(value)) return null;
  const x =
    toNumber(value.x) ??
    toNumber(value.xMeters) ??
    (isObject(value.translation) ? toNumber(value.translation.x) ?? toNumber(value.translation.xMeters) : null);
  const y =
    toNumber(value.y) ??
    toNumber(value.yMeters) ??
    (isObject(value.translation) ? toNumber(value.translation.y) ?? toNumber(value.translation.yMeters) : null);
  if (x !== null && y !== null) {
    return { x, y };
  }

  return null;
}

export function trajectoryCandidatesFromUnknown(value: unknown, depth = 0): unknown[] {
  if (depth > 2) return [];

  if (typeof value === 'string') {
    const trimmed = value.trim();
    if (!trimmed) return [];
    try {
      return trajectoryCandidatesFromUnknown(JSON.parse(trimmed) as unknown, depth + 1);
    } catch {
      return [];
    }
  }

  if (Array.isArray(value)) return value;
  if (!isObject(value)) return [];

  if (Array.isArray(value.points)) return value.points;
  if (Array.isArray(value.trajectory)) return value.trajectory;
  if (Array.isArray(value.path)) return value.path;
  if (Array.isArray(value.poses)) return value.poses;
  if (Array.isArray(value.samples)) return value.samples;
  if (Array.isArray(value.waypoints)) return value.waypoints;
  if (pointFromUnknown(value) !== null) return [value];

  return [];
}

export function parseTrajectoryPoints(raw: string): FieldPoint[] {
  const trimmed = raw.trim();
  if (!trimmed) return [];

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    const candidates = trajectoryCandidatesFromUnknown(parsed);

    return candidates
      .map((entry) => pointFromUnknown(entry))
      .filter((entry): entry is FieldPoint => entry !== null)
      .slice(0, 1000);
  } catch {
    // Fallback: parse "x,y;x,y" representation.
  }

  const points = trimmed
    .split(';')
    .map((pair) => pair.trim())
    .filter((pair) => pair.length > 0)
    .map((pair) => {
      const [xRaw, yRaw] = pair.split(',');
      const x = toNumber(xRaw);
      const y = toNumber(yRaw);
      if (x === null || y === null) return null;
      return { x, y };
    })
    .filter((entry): entry is FieldPoint => entry !== null);

  return points.slice(0, 1000);
}
