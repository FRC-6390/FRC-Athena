import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, isNtPublishToggleSignal, isObject, normalizeSignalId, normalizeToken, parentPath, resolveConfiguredSignalId, resolveSignalIdFromPath, toBoolean, toString } from './common';

export type DioWidgetConfig = {
  showOtherFields: boolean;
  valueSignalId: number | null;
  outputSignalId: number | null;
  invertedSignalId: number | null;
  channelSignalId: number | null;
  portSignalId: number | null;
  modeSignalId: number | null;
  nameSignalId: number | null;
  typeSignalId: number | null;
};

export function findWritableBoolSignalByLeaf(
  signals: SignalRow[],
  anchorParent: string,
  candidates: string[]
): SignalRow | null {
  const scoped = signals.filter((signal) => parentPath(signal.path) === anchorParent);
  const pool = scoped.length > 0 ? scoped : signals;
  const normalizedCandidates = candidates.map((candidate) => normalizeToken(candidate));

  for (const signal of pool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (signal.signal_type !== 'bool' || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.includes(token)) {
      return signal;
    }
  }

  for (const signal of pool) {
    if (isNtPublishToggleSignal(signal)) continue;
    if (signal.signal_type !== 'bool' || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.some((candidate) => token.endsWith(candidate) || token.includes(candidate))) {
      return signal;
    }
  }

  return null;
}

export function defaultDioConfig(anchor: SignalRow, signals: SignalRow[]): DioWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const valueSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['value', 'state', 'pressed', 'triggered', 'blocked', 'active', 'detected'],
    ['bool']
  );
  const outputSignal = findWritableBoolSignalByLeaf(
    signals,
    anchorParent,
    ['output', 'set', 'command', 'enabled', 'state', 'value']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );
  const channelSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['channel', 'dio_channel', 'diochannel'],
    ['i64', 'f64']
  );
  const portSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['port', 'dio_port'],
    ['i64', 'f64']
  );
  const modeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['mode', 'direction'],
    ['string', 'bool', 'i64', 'f64']
  );
  const nameSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['name', 'label'],
    ['string']
  );
  const typeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['type', 'sensor_type'],
    ['string']
  );

  const leaf = normalizeToken(leafPath(anchor.path));
  const fallbackValueSignalId =
    anchor.signal_type === 'bool' &&
    ['value', 'state', 'pressed', 'triggered', 'blocked', 'active', 'detected'].some((token) =>
      leaf.includes(normalizeToken(token))
    )
      ? anchor.signal_id
      : null;

  return {
    showOtherFields: true,
    valueSignalId: valueSignal?.signal_id ?? fallbackValueSignalId,
    outputSignalId: outputSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    channelSignalId: channelSignal?.signal_id ?? null,
    portSignalId: portSignal?.signal_id ?? null,
    modeSignalId: modeSignal?.signal_id ?? null,
    nameSignalId: nameSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null
  };
}

export function readDioConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): DioWidgetConfig {
  const defaults = defaultDioConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const topicPath = toString(base.topicPath, '').trim();

  const configuredChannelPath = toString(
    base.channelSignalPath,
    toString(base.dioChannelSignalPath, '')
  ).trim();
  const channelSignalFromPath = configuredChannelPath
    ? resolveSignalIdFromPath(configuredChannelPath, topicPath, fallbackSignal, signals)
    : null;
  const configuredChannelSignalId =
    normalizeSignalId(base.channelSignalId) ?? normalizeSignalId(base.dioChannelSignalId);
  const channelSignalId =
    channelSignalFromPath ??
    (configuredChannelSignalId !== null &&
    signals.some((entry) => entry.signal_id === configuredChannelSignalId)
      ? configuredChannelSignalId
      : null) ??
    defaults.channelSignalId;

  return {
    showOtherFields: toBoolean(base.showOtherFields, defaults.showOtherFields),
    valueSignalId: resolveConfiguredSignalId(
      base,
      'valueSignalId',
      'valueSignalPath',
      defaults.valueSignalId,
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
    invertedSignalId: resolveConfiguredSignalId(
      base,
      'invertedSignalId',
      'invertedSignalPath',
      defaults.invertedSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    channelSignalId,
    portSignalId: resolveConfiguredSignalId(
      base,
      'portSignalId',
      'portSignalPath',
      defaults.portSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    modeSignalId: resolveConfiguredSignalId(
      base,
      'modeSignalId',
      'modeSignalPath',
      defaults.modeSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    nameSignalId: resolveConfiguredSignalId(
      base,
      'nameSignalId',
      'nameSignalPath',
      defaults.nameSignalId,
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
    )
  };
}
