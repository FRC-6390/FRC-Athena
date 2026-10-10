import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, isNumericSignal, isObject, normalizeToken, parentPath, resolveConfiguredSignalId, toNumber, toString } from './common';

export type EncoderWidgetConfig = {
  positionSignalId: number | null;
  velocitySignalId: number | null;
  rateSignalId: number | null;
  absoluteSignalId: number | null;
  connectedSignalId: number | null;
  conversionSignalId: number | null;
  conversionOffsetSignalId: number | null;
  discontinuityPointSignalId: number | null;
  discontinuityRangeSignalId: number | null;
  ratioSignalId: number | null;
  offsetSignalId: number | null;
  canIdSignalId: number | null;
  canbusSignalId: number | null;
  typeSignalId: number | null;
  invertedSignalId: number | null;
  supportsSimulationSignalId: number | null;
  rawAbsoluteSignalId: number | null;
  positionViewMode: 'continuous' | 'zero_to_one' | 'neg180_to_180' | 'zero_to_360' | 'distance';
  absoluteViewMode: 'continuous' | 'zero_to_one' | 'neg180_to_180' | 'zero_to_360' | 'distance';
  positionMin: number;
  positionMax: number;
  absoluteMin: number;
  absoluteMax: number;
  positionUnit: string;
  absoluteUnit: string;
};

export function defaultEncoderConfig(anchor: SignalRow, signals: SignalRow[]): EncoderWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const positionSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['position', 'pos', 'rotations', 'distance'],
    ['f64', 'i64']
  );
  const velocitySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['velocity', 'speed', 'rate', 'rpm'],
    ['f64', 'i64']
  );
  const rateSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['rate', 'rps'],
    ['f64', 'i64']
  );
  const absoluteSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['absolute', 'absolute_position', 'abs'],
    ['f64', 'i64']
  );
  const connectedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['connected', 'is_connected', 'healthy', 'online'],
    ['bool']
  );
  const conversionSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['conversion', 'conversion_ratio', 'conversion_factor', 'rotations_per_unit'],
    ['f64', 'i64']
  );
  const conversionOffsetSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['conversion_offset', 'conversionoffset', 'offset_conversion', 'conversion_zero_offset'],
    ['f64', 'i64']
  );
  const discontinuityPointSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['discontinuity_point', 'discontinuitypoint'],
    ['f64', 'i64']
  );
  const discontinuityRangeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['discontinuity_range', 'discontinuityrange'],
    ['f64', 'i64']
  );
  const ratioSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['ratio', 'gear_ratio', 'conversion'],
    ['f64', 'i64']
  );
  const offsetSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['offset', 'zero_offset', 'calibration_offset'],
    ['f64', 'i64']
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
    ['type', 'encoder_type', 'sensor_type'],
    ['string']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );
  const supportsSimulationSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['supports_simulation', 'sim_supported', 'simulation'],
    ['bool']
  );
  const rawAbsoluteSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['raw_absolute_rot', 'raw_absolute', 'raw'],
    ['f64', 'i64']
  );

  const fallbackPosition =
    isNumericSignal(anchor) &&
    ['position', 'rotations', 'distance'].some((token) =>
      normalizeToken(leafPath(anchor.path)).includes(normalizeToken(token))
    )
      ? anchor.signal_id
      : null;

  return {
    positionSignalId: positionSignal?.signal_id ?? fallbackPosition,
    velocitySignalId: velocitySignal?.signal_id ?? null,
    rateSignalId: rateSignal?.signal_id ?? null,
    absoluteSignalId: absoluteSignal?.signal_id ?? null,
    connectedSignalId: connectedSignal?.signal_id ?? null,
    conversionSignalId: conversionSignal?.signal_id ?? null,
    conversionOffsetSignalId: conversionOffsetSignal?.signal_id ?? null,
    discontinuityPointSignalId: discontinuityPointSignal?.signal_id ?? null,
    discontinuityRangeSignalId: discontinuityRangeSignal?.signal_id ?? null,
    ratioSignalId: ratioSignal?.signal_id ?? null,
    offsetSignalId: offsetSignal?.signal_id ?? null,
    canIdSignalId: canIdSignal?.signal_id ?? null,
    canbusSignalId: canbusSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    supportsSimulationSignalId: supportsSimulationSignal?.signal_id ?? null,
    rawAbsoluteSignalId: rawAbsoluteSignal?.signal_id ?? null,
    positionViewMode: 'continuous',
    absoluteViewMode: 'zero_to_one',
    positionMin: -2.0,
    positionMax: 2.0,
    absoluteMin: 0.0,
    absoluteMax: 1.0,
    positionUnit: '',
    absoluteUnit: ''
  };
}

export function readEncoderConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): EncoderWidgetConfig {
  const defaults = defaultEncoderConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const topicPath = toString(base.topicPath, '').trim();

  return {
    positionSignalId: resolveConfiguredSignalId(
      base,
      'positionSignalId',
      'positionSignalPath',
      defaults.positionSignalId,
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
    rateSignalId: resolveConfiguredSignalId(
      base,
      'rateSignalId',
      'rateSignalPath',
      defaults.rateSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    absoluteSignalId: resolveConfiguredSignalId(
      base,
      'absoluteSignalId',
      'absoluteSignalPath',
      defaults.absoluteSignalId,
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
    conversionSignalId: resolveConfiguredSignalId(
      base,
      'conversionSignalId',
      'conversionSignalPath',
      defaults.conversionSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    conversionOffsetSignalId: resolveConfiguredSignalId(
      base,
      'conversionOffsetSignalId',
      'conversionOffsetSignalPath',
      defaults.conversionOffsetSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    discontinuityPointSignalId: resolveConfiguredSignalId(
      base,
      'discontinuityPointSignalId',
      'discontinuityPointSignalPath',
      defaults.discontinuityPointSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    discontinuityRangeSignalId: resolveConfiguredSignalId(
      base,
      'discontinuityRangeSignalId',
      'discontinuityRangeSignalPath',
      defaults.discontinuityRangeSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    ratioSignalId: resolveConfiguredSignalId(
      base,
      'ratioSignalId',
      'ratioSignalPath',
      defaults.ratioSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    offsetSignalId: resolveConfiguredSignalId(
      base,
      'offsetSignalId',
      'offsetSignalPath',
      defaults.offsetSignalId,
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
    supportsSimulationSignalId: resolveConfiguredSignalId(
      base,
      'supportsSimulationSignalId',
      'supportsSimulationSignalPath',
      defaults.supportsSimulationSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    rawAbsoluteSignalId: resolveConfiguredSignalId(
      base,
      'rawAbsoluteSignalId',
      'rawAbsoluteSignalPath',
      defaults.rawAbsoluteSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    positionViewMode:
      base.positionViewMode === 'zero_to_one' ||
      base.positionViewMode === 'continuous' ||
      base.positionViewMode === 'neg180_to_180' ||
      base.positionViewMode === 'zero_to_360' ||
      base.positionViewMode === 'distance'
        ? base.positionViewMode
        : defaults.positionViewMode,
    absoluteViewMode:
      base.absoluteViewMode === 'zero_to_one' ||
      base.absoluteViewMode === 'continuous' ||
      base.absoluteViewMode === 'neg180_to_180' ||
      base.absoluteViewMode === 'zero_to_360' ||
      base.absoluteViewMode === 'distance'
        ? base.absoluteViewMode
        : defaults.absoluteViewMode,
    positionMin: toNumber(base.positionMin) ?? defaults.positionMin,
    positionMax: toNumber(base.positionMax) ?? defaults.positionMax,
    absoluteMin: toNumber(base.absoluteMin) ?? defaults.absoluteMin,
    absoluteMax: toNumber(base.absoluteMax) ?? defaults.absoluteMax,
    positionUnit: toString(base.positionUnit, defaults.positionUnit),
    absoluteUnit: toString(base.absoluteUnit, defaults.absoluteUnit)
  };
}
