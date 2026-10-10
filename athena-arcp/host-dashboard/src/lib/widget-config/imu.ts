import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, isNumericSignal, isObject, normalizeToken, parentPath, resolveConfiguredSignalId, resolveSignalIdFromKeys, toString } from './common';

export type Imu3dUnits = 'deg' | 'rad';

export type ImuViewMode = 'auto' | '1d' | '2d' | '3d';

export type Imu3dWidgetConfig = {
  rollSignalId: number | null;
  pitchSignalId: number | null;
  yawSignalId: number | null;
  units: Imu3dUnits;
};

export type ImuWidgetConfig = {
  rollSignalId: number | null;
  pitchSignalId: number | null;
  yawSignalId: number | null;
  headingSignalId: number | null;
  connectedSignalId: number | null;
  accelSignalId: number | null;
  gyroSignalId: number | null;
  magSignalId: number | null;
  canIdSignalId: number | null;
  canbusSignalId: number | null;
  typeSignalId: number | null;
  invertedSignalId: number | null;
  orientationViewMode: ImuViewMode;
  accelViewMode: ImuViewMode;
  gyroViewMode: ImuViewMode;
  magViewMode: ImuViewMode;
  units: Imu3dUnits;
};

export function defaultImu3dConfig(anchor: SignalRow, signals: SignalRow[]): Imu3dWidgetConfig {
  const anchorParent = parentPath(anchor.path);
  const rollSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['roll', 'phi', 'bank'],
    ['f64', 'i64']
  );
  const pitchSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['pitch', 'theta', 'attitude'],
    ['f64', 'i64']
  );
  const yawSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['yaw', 'heading', 'psi', 'gyro', 'angle', 'bearing'],
    ['f64', 'i64']
  );

  const leaf = normalizeToken(leafPath(anchor.path));
  const rollSignalId =
    rollSignal?.signal_id ??
    (isNumericSignal(anchor) && ['roll', 'phi', 'bank'].some((token) => leaf.endsWith(token))
      ? anchor.signal_id
      : null);
  const pitchSignalId =
    pitchSignal?.signal_id ??
    (isNumericSignal(anchor) && ['pitch', 'theta', 'attitude'].some((token) => leaf.endsWith(token))
      ? anchor.signal_id
      : null);
  const yawSignalId =
    yawSignal?.signal_id ??
    (isNumericSignal(anchor) && ['yaw', 'heading', 'psi', 'bearing'].some((token) => leaf.endsWith(token))
      ? anchor.signal_id
      : null);

  const units: Imu3dUnits = anchor.path.toLowerCase().includes('rad') ? 'rad' : 'deg';

  return {
    rollSignalId,
    pitchSignalId,
    yawSignalId,
    units
  };
}

export function readImu3dConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): Imu3dWidgetConfig {
  const defaults = defaultImu3dConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const unitsRaw = toString(base.units, defaults.units).toLowerCase();
  const units: Imu3dUnits = unitsRaw === 'rad' ? 'rad' : 'deg';

  return {
    rollSignalId: resolveSignalIdFromKeys(base, ['rollSignalId'], defaults.rollSignalId, signals),
    pitchSignalId: resolveSignalIdFromKeys(base, ['pitchSignalId'], defaults.pitchSignalId, signals),
    yawSignalId: resolveSignalIdFromKeys(base, ['yawSignalId'], defaults.yawSignalId, signals),
    units
  };
}

export function defaultImuConfig(anchor: SignalRow, signals: SignalRow[]): ImuWidgetConfig {
  const baseImu3d = defaultImu3dConfig(anchor, signals);
  const anchorParent = parentPath(anchor.path);

  const headingSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['heading', 'yaw', 'bearing', 'angle', 'gyro'],
    ['f64', 'i64']
  );
  const connectedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['connected', 'is_connected', 'healthy', 'online'],
    ['bool']
  );
  const accelSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['accel', 'acceleration', 'linear_accel'],
    ['f64[]', 'i64[]']
  );
  const gyroSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['gyro_rates', 'gyro', 'angular_velocity', 'rates'],
    ['f64[]', 'i64[]']
  );
  const magSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['mag_xyz_ut', 'mag_xyz', 'magnetometer', 'magnetic_field', 'mag'],
    ['f64[]', 'i64[]']
  );
  const canIdSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['canid', 'can_id', 'id'],
    ['f64', 'i64']
  );
  const canbusSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['canbus', 'bus'],
    ['string']
  );
  const typeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['type', 'imu_type', 'sensor_type'],
    ['string']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );

  return {
    rollSignalId: baseImu3d.rollSignalId,
    pitchSignalId: baseImu3d.pitchSignalId,
    yawSignalId: baseImu3d.yawSignalId,
    headingSignalId: headingSignal?.signal_id ?? baseImu3d.yawSignalId,
    connectedSignalId: connectedSignal?.signal_id ?? null,
    accelSignalId: accelSignal?.signal_id ?? null,
    gyroSignalId: gyroSignal?.signal_id ?? null,
    magSignalId: magSignal?.signal_id ?? null,
    canIdSignalId: canIdSignal?.signal_id ?? null,
    canbusSignalId: canbusSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    orientationViewMode: 'auto',
    accelViewMode: 'auto',
    gyroViewMode: 'auto',
    magViewMode: 'auto',
    units: baseImu3d.units
  };
}

export function readImuConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): ImuWidgetConfig {
  const defaults = defaultImuConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const topicPath = toString(base.topicPath, '').trim();
  const unitsRaw = toString(base.units, defaults.units).toLowerCase();
  const units: Imu3dUnits = unitsRaw === 'rad' ? 'rad' : 'deg';

  return {
    rollSignalId: resolveConfiguredSignalId(
      base,
      'rollSignalId',
      'rollSignalPath',
      defaults.rollSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    pitchSignalId: resolveConfiguredSignalId(
      base,
      'pitchSignalId',
      'pitchSignalPath',
      defaults.pitchSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    yawSignalId: resolveConfiguredSignalId(
      base,
      'yawSignalId',
      'yawSignalPath',
      defaults.yawSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    headingSignalId: resolveConfiguredSignalId(
      base,
      'headingSignalId',
      'headingSignalPath',
      defaults.headingSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    connectedSignalId: resolveConfiguredSignalId(
      base,
      'connectedSignalId',
      'connectedSignalPath',
      defaults.connectedSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    accelSignalId: resolveConfiguredSignalId(
      base,
      'accelSignalId',
      'accelSignalPath',
      defaults.accelSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    gyroSignalId: resolveConfiguredSignalId(
      base,
      'gyroSignalId',
      'gyroSignalPath',
      defaults.gyroSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    magSignalId: resolveConfiguredSignalId(
      base,
      'magSignalId',
      'magSignalPath',
      defaults.magSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    canIdSignalId: resolveConfiguredSignalId(
      base,
      'canIdSignalId',
      'canIdSignalPath',
      defaults.canIdSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    canbusSignalId: resolveConfiguredSignalId(
      base,
      'canbusSignalId',
      'canbusSignalPath',
      defaults.canbusSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    typeSignalId: resolveConfiguredSignalId(
      base,
      'typeSignalId',
      'typeSignalPath',
      defaults.typeSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    invertedSignalId: resolveConfiguredSignalId(
      base,
      'invertedSignalId',
      'invertedSignalPath',
      defaults.invertedSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    orientationViewMode:
      base.orientationViewMode === '1d' ||
      base.orientationViewMode === '2d' ||
      base.orientationViewMode === '3d' ||
      base.orientationViewMode === 'auto'
        ? base.orientationViewMode
        : defaults.orientationViewMode,
    accelViewMode:
      base.accelViewMode === '1d' ||
      base.accelViewMode === '2d' ||
      base.accelViewMode === '3d' ||
      base.accelViewMode === 'auto'
        ? base.accelViewMode
        : defaults.accelViewMode,
    gyroViewMode:
      base.gyroViewMode === '1d' ||
      base.gyroViewMode === '2d' ||
      base.gyroViewMode === '3d' ||
      base.gyroViewMode === 'auto'
        ? base.gyroViewMode
        : defaults.gyroViewMode,
    magViewMode:
      base.magViewMode === '1d' ||
      base.magViewMode === '2d' ||
      base.magViewMode === '3d' ||
      base.magViewMode === 'auto'
        ? base.magViewMode
        : defaults.magViewMode,
    units
  };
}
