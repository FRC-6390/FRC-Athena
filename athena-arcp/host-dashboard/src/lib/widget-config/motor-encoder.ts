import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, findWritableNumericSignalByLeaf, isNumericSignal, isObject, normalizeSignalId, normalizeToken, parentPath, resolveConfiguredSignalId, resolveSignalIdFromPath, toBoolean, toString } from './common';

export type MotorWidgetConfig = {
  showOtherFields: boolean;
  canIdSignalId: number | null;
  canbusSignalId: number | null;
  typeSignalId: number | null;
  connectedSignalId: number | null;
  stalledSignalId: number | null;
  neutralModeSignalId: number | null;
  currentLimitSignalId: number | null;
  invertedSignalId: number | null;
  brakeModeSignalId: number | null;
  outputSignalId: number | null;
  velocitySignalId: number | null;
  positionSignalId: number | null;
  currentSignalId: number | null;
  temperatureSignalId: number | null;
  voltageSignalId: number | null;
  commandSignalId: number | null;
};

export function defaultMotorConfig(anchor: SignalRow, signals: SignalRow[]): MotorWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const canIdSignal = findSignalByLeaf(signals, anchorParent, ['canid', 'can_id', 'id'], ['f64', 'i64']);
  const canbusSignal = findSignalByLeaf(signals, anchorParent, ['canbus', 'bus'], ['string']);
  const typeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['type', 'motortype', 'controller', 'controller_type'],
    ['string']
  );
  const connectedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['connected', 'is_connected', 'alive', 'online', 'faulted'],
    ['bool']
  );
  const stalledSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['stalled', 'stall', 'is_stalled'],
    ['bool']
  );
  const neutralModeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['neutral_mode', 'neutralmode', 'idle_mode', 'idlemode', 'mode'],
    ['string', 'bool']
  );
  const currentLimitSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['current_limit_a', 'currentlimita', 'current_limit', 'limit_amps', 'max_current'],
    ['f64', 'i64']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );
  const brakeModeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['brake_mode', 'brakemode'],
    ['bool', 'string']
  );
  const outputSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['output', 'percent_output', 'duty_cycle', 'applied_output', 'command', 'set'],
    ['f64', 'i64']
  );
  const velocitySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['velocity', 'speed', 'rpm', 'rate'],
    ['f64', 'i64']
  );
  const positionSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['position', 'rotations', 'distance', 'pos'],
    ['f64', 'i64']
  );
  const currentSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['current', 'amps', 'supply_current', 'stator_current'],
    ['f64', 'i64']
  );
  const temperatureSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['temperature', 'temp', 'temp_c', 'motor_temp'],
    ['f64', 'i64']
  );
  const voltageSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['voltage', 'bus_voltage', 'output_voltage'],
    ['f64', 'i64']
  );
  const commandSignal = findWritableNumericSignalByLeaf(
    signals,
    anchorParent,
    ['output', 'set', 'command', 'speed', 'velocity', 'voltage']
  );

  const fallbackOutput =
    isNumericSignal(anchor) &&
    ['output', 'command', 'set', 'speed', 'velocity'].some((token) =>
      normalizeToken(leafPath(anchor.path)).includes(normalizeToken(token))
    )
      ? anchor.signal_id
      : null;

  return {
    showOtherFields: true,
    canIdSignalId: canIdSignal?.signal_id ?? null,
    canbusSignalId: canbusSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null,
    connectedSignalId: connectedSignal?.signal_id ?? null,
    stalledSignalId: stalledSignal?.signal_id ?? null,
    neutralModeSignalId: neutralModeSignal?.signal_id ?? brakeModeSignal?.signal_id ?? null,
    currentLimitSignalId: currentLimitSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    brakeModeSignalId: brakeModeSignal?.signal_id ?? null,
    outputSignalId: outputSignal?.signal_id ?? fallbackOutput,
    velocitySignalId: velocitySignal?.signal_id ?? null,
    positionSignalId: positionSignal?.signal_id ?? null,
    currentSignalId: currentSignal?.signal_id ?? null,
    temperatureSignalId: temperatureSignal?.signal_id ?? null,
    voltageSignalId: voltageSignal?.signal_id ?? null,
    commandSignalId: commandSignal?.signal_id ?? null
  };
}

export function readMotorConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): MotorWidgetConfig {
  const defaults = defaultMotorConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const topicPath = toString(base.topicPath, '').trim();

  const configuredNeutralModePath = toString(
    base.neutralModeSignalPath,
    toString(base.modeSignalPath, '')
  ).trim();
  const neutralModeFromPath = configuredNeutralModePath
    ? resolveSignalIdFromPath(configuredNeutralModePath, topicPath, fallbackSignal, signals)
    : null;
  const configuredNeutralModeSignalId =
    normalizeSignalId(base.neutralModeSignalId) ?? normalizeSignalId(base.modeSignalId);
  const neutralModeSignalId =
    neutralModeFromPath ??
    (configuredNeutralModeSignalId !== null &&
    signals.some((entry) => entry.signal_id === configuredNeutralModeSignalId)
      ? configuredNeutralModeSignalId
      : null) ??
    defaults.neutralModeSignalId;

  return {
    showOtherFields: toBoolean(base.showOtherFields, defaults.showOtherFields),
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
    connectedSignalId: resolveConfiguredSignalId(
      base,
      'connectedSignalId',
      'connectedSignalPath',
      defaults.connectedSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    stalledSignalId: resolveConfiguredSignalId(
      base,
      'stalledSignalId',
      'stalledSignalPath',
      defaults.stalledSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    neutralModeSignalId,
    currentLimitSignalId: resolveConfiguredSignalId(
      base,
      'currentLimitSignalId',
      'currentLimitSignalPath',
      defaults.currentLimitSignalId,
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
    brakeModeSignalId: resolveConfiguredSignalId(
      base,
      'brakeModeSignalId',
      'brakeModeSignalPath',
      defaults.brakeModeSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    outputSignalId: resolveConfiguredSignalId(
      base,
      'outputSignalId',
      'outputSignalPath',
      defaults.outputSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    velocitySignalId: resolveConfiguredSignalId(
      base,
      'velocitySignalId',
      'velocitySignalPath',
      defaults.velocitySignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    positionSignalId: resolveConfiguredSignalId(
      base,
      'positionSignalId',
      'positionSignalPath',
      defaults.positionSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    currentSignalId: resolveConfiguredSignalId(
      base,
      'currentSignalId',
      'currentSignalPath',
      defaults.currentSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    temperatureSignalId: resolveConfiguredSignalId(
      base,
      'temperatureSignalId',
      'temperatureSignalPath',
      defaults.temperatureSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    voltageSignalId: resolveConfiguredSignalId(
      base,
      'voltageSignalId',
      'voltageSignalPath',
      defaults.voltageSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    commandSignalId: resolveConfiguredSignalId(
      base,
      'commandSignalId',
      'commandSignalPath',
      defaults.commandSignalId,
      topicPath,
      fallbackSignal,
      signals
    )
  };
}
