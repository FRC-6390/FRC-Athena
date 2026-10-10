import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, isNumericSignal, isObject, normalizeToken, parentPath, resolveConfiguredSignalId, resolveSignalIdFromKeys, toNumber, toString } from './common';

export type SwerveModuleWidgetConfig = {
  angleSignalId: number | null;
  speedSignalId: number | null;
  commandAngleSignalId: number | null;
  commandSpeedSignalId: number | null;
  commandInvertedSignalId: number | null;
  commandOffsetSignalId: number | null;
  label: string;
  maxSpeed: number;
};

export type SwerveDriveModuleBinding = {
  key: string;
  label: string;
  angleSignalId: number | null;
  speedSignalId: number | null;
};

export type SwerveDriveWidgetConfig = {
  modules: SwerveDriveModuleBinding[];
  headingSignalId: number | null;
  maxSpeed: number;
};

export type DifferentialDriveWidgetConfig = {
  leftSpeedSignalId: number | null;
  rightSpeedSignalId: number | null;
  headingSignalId: number | null;
  maxSpeed: number;
};

export const SWERVE_MODULE_KEYS = ['fl', 'fr', 'bl', 'br'] as const;

export const SWERVE_ANGLE_ALIASES = ['angle', 'angledeg', 'heading', 'yaw', 'theta', 'azimuth', 'steer'];

export const SWERVE_SPEED_ALIASES = ['speed', 'speedmps', 'velocity', 'vel', 'drive', 'drivespeed'];

export function moduleTokenFromPath(path: string): string | null {
  const parts = path.toLowerCase().split('/').filter(Boolean);
  for (const part of parts) {
    if ((SWERVE_MODULE_KEYS as readonly string[]).includes(part)) {
      return part;
    }
  }
  return null;
}

export function hasAlias(token: string, aliases: readonly string[]): boolean {
  return aliases.some((alias) => token === alias || token.endsWith(alias));
}

export function findSwerveSignal(
  signals: SignalRow[],
  moduleKey: string,
  aliases: readonly string[]
): SignalRow | null {
  const modulePathToken = `/swerve/${moduleKey}/`;
  const inModule = signals.filter((signal) => {
    if (!isNumericSignal(signal)) return false;
    return signal.path.toLowerCase().includes(modulePathToken);
  });

  for (const signal of inModule) {
    const leaf = normalizeToken(leafPath(signal.path));
    if (hasAlias(leaf, aliases)) {
      return signal;
    }
  }

  for (const signal of inModule) {
    const leaf = normalizeToken(leafPath(signal.path));
    if (aliases.some((alias) => leaf.includes(alias))) {
      return signal;
    }
  }

  return null;
}

export function defaultSwerveModuleConfig(
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveModuleWidgetConfig {
  const moduleKey = moduleTokenFromPath(fallbackSignal.path);
  let angleSignal = moduleKey ? findSwerveSignal(signals, moduleKey, SWERVE_ANGLE_ALIASES) : null;
  let speedSignal = moduleKey ? findSwerveSignal(signals, moduleKey, SWERVE_SPEED_ALIASES) : null;

  if (!angleSignal || !speedSignal) {
    const parent = parentPath(fallbackSignal.path);
    angleSignal =
      angleSignal ??
      findSignalByLeaf(signals, parent, SWERVE_ANGLE_ALIASES, ['f64', 'i64']);
    speedSignal =
      speedSignal ??
      findSignalByLeaf(signals, parent, SWERVE_SPEED_ALIASES, ['f64', 'i64']);
  }

  if (isNumericSignal(fallbackSignal)) {
    const leaf = normalizeToken(leafPath(fallbackSignal.path));
    if (!angleSignal && hasAlias(leaf, SWERVE_ANGLE_ALIASES)) {
      angleSignal = fallbackSignal;
    }
    if (!speedSignal && hasAlias(leaf, SWERVE_SPEED_ALIASES)) {
      speedSignal = fallbackSignal;
    }
  }

  const fallbackLabel = moduleKey
    ? moduleKey.toUpperCase()
    : leafPath(parentPath(fallbackSignal.path) || fallbackSignal.path).toUpperCase();

  return {
    angleSignalId: angleSignal?.signal_id ?? null,
    speedSignalId: speedSignal?.signal_id ?? null,
    commandAngleSignalId: null,
    commandSpeedSignalId: null,
    commandInvertedSignalId: null,
    commandOffsetSignalId: null,
    label: fallbackLabel,
    maxSpeed: 5
  };
}

export function readSwerveModuleConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveModuleWidgetConfig {
  const defaults = defaultSwerveModuleConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    angleSignalId: resolveConfiguredSignalId(
      base,
      'angleSignalId',
      'angleSignalPath',
      defaults.angleSignalId,
      '',
      fallbackSignal,
      signals
    ),
    speedSignalId: resolveConfiguredSignalId(
      base,
      'speedSignalId',
      'speedSignalPath',
      defaults.speedSignalId,
      '',
      fallbackSignal,
      signals
    ),
    commandAngleSignalId: resolveConfiguredSignalId(
      base,
      'commandAngleSignalId',
      'commandAngleSignalPath',
      defaults.commandAngleSignalId,
      '',
      fallbackSignal,
      signals
    ),
    commandSpeedSignalId: resolveConfiguredSignalId(
      base,
      'commandSpeedSignalId',
      'commandSpeedSignalPath',
      defaults.commandSpeedSignalId,
      '',
      fallbackSignal,
      signals
    ),
    commandInvertedSignalId: resolveConfiguredSignalId(
      base,
      'commandInvertedSignalId',
      'commandInvertedSignalPath',
      defaults.commandInvertedSignalId,
      '',
      fallbackSignal,
      signals
    ),
    commandOffsetSignalId: resolveConfiguredSignalId(
      base,
      'commandOffsetSignalId',
      'commandOffsetSignalPath',
      defaults.commandOffsetSignalId,
      '',
      fallbackSignal,
      signals
    ),
    label: toString(base.label, defaults.label).trim() || defaults.label,
    maxSpeed: Math.max(0.1, toNumber(base.maxSpeed) ?? defaults.maxSpeed)
  };
}

export function defaultSwerveDriveConfig(
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveDriveWidgetConfig {
  const modules = SWERVE_MODULE_KEYS.map((key) => {
    const angleSignal = findSwerveSignal(signals, key, SWERVE_ANGLE_ALIASES);
    const speedSignal = findSwerveSignal(signals, key, SWERVE_SPEED_ALIASES);
    return {
      key,
      label: key.toUpperCase(),
      angleSignalId: angleSignal?.signal_id ?? null,
      speedSignalId: speedSignal?.signal_id ?? null
    };
  });

  const headingSignal =
    findSignalByLeaf(
      signals,
      parentPath(fallbackSignal.path),
      ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/swerve/') && path.endsWith('/heading_deg');
    }) ??
    null;

  return {
    modules,
    headingSignalId: headingSignal?.signal_id ?? null,
    maxSpeed: 5
  };
}

export function readSwerveDriveConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveDriveWidgetConfig {
  const defaults = defaultSwerveDriveConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  const modules = [...defaults.modules];
  if (Array.isArray(base.modules)) {
    for (const entry of base.modules) {
      if (!isObject(entry)) continue;
      const key = toString(entry.key, '').toLowerCase();
      const idx = modules.findIndex((module) => module.key === key);
      if (idx < 0) continue;
      modules[idx] = {
        ...modules[idx],
        label: toString(entry.label, modules[idx].label).trim() || modules[idx].label,
        angleSignalId: resolveSignalIdFromKeys(
          entry,
          ['angleSignalId'],
          modules[idx].angleSignalId,
          signals
        ),
        speedSignalId: resolveSignalIdFromKeys(
          entry,
          ['speedSignalId'],
          modules[idx].speedSignalId,
          signals
        )
      };
    }
  }

  return {
    modules,
    headingSignalId: resolveSignalIdFromKeys(
      base,
      ['headingSignalId'],
      defaults.headingSignalId,
      signals
    ),
    maxSpeed: Math.max(0.1, toNumber(base.maxSpeed) ?? defaults.maxSpeed)
  };
}

export function defaultDifferentialDriveConfig(
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): DifferentialDriveWidgetConfig {
  const parent = parentPath(fallbackSignal.path);
  const left =
    findSignalByLeaf(
      signals,
      parent,
      ['leftspeed', 'left_speed', 'leftvelocity', 'left_vel', 'left'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/diff/') && path.includes('left') && path.includes('speed');
    }) ??
    null;
  const right =
    findSignalByLeaf(
      signals,
      parent,
      ['rightspeed', 'right_speed', 'rightvelocity', 'right_vel', 'right'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/diff/') && path.includes('right') && path.includes('speed');
    }) ??
    null;
  const heading =
    findSignalByLeaf(
      signals,
      parent,
      ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/diff/') && path.includes('heading');
    }) ??
    null;

  return {
    leftSpeedSignalId: left?.signal_id ?? null,
    rightSpeedSignalId: right?.signal_id ?? null,
    headingSignalId: heading?.signal_id ?? null,
    maxSpeed: 5
  };
}

export function readDifferentialDriveConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): DifferentialDriveWidgetConfig {
  const defaults = defaultDifferentialDriveConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    leftSpeedSignalId: resolveSignalIdFromKeys(
      base,
      ['leftSpeedSignalId'],
      defaults.leftSpeedSignalId,
      signals
    ),
    rightSpeedSignalId: resolveSignalIdFromKeys(
      base,
      ['rightSpeedSignalId'],
      defaults.rightSpeedSignalId,
      signals
    ),
    headingSignalId: resolveSignalIdFromKeys(
      base,
      ['headingSignalId'],
      defaults.headingSignalId,
      signals
    ),
    maxSpeed: Math.max(0.1, toNumber(base.maxSpeed) ?? defaults.maxSpeed)
  };
}
