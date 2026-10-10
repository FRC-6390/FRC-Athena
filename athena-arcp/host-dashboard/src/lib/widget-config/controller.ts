import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, isNtPublishToggleSignal, isNumericSignal, isObject, normalizeToken, parentPath, resolveSignalIdFromKeys, toBoolean, toNumber, toString } from './common';
import { findActionSignalByLeaf } from './state-machine';

export const CONTROLLER_PARAM_ORDER = [
  'kp',
  'ki',
  'kd',
  'ks',
  'kv',
  'ka',
  'kg',
  'izone',
  'setpoint'
] as const;

export const CONTROLLER_PARAM_ALIASES: Record<string, string[]> = {
  kp: ['kp', 'p', 'proportional'],
  ki: ['ki', 'i', 'integral'],
  kd: ['kd', 'd', 'derivative'],
  ks: ['ks', 's', 'static'],
  kv: ['kv', 'v', 'velocity'],
  ka: ['ka', 'a', 'accel', 'acceleration'],
  kg: ['kg', 'g', 'gravity'],
  izone: ['izone', 'integralzone', 'iz'],
  setpoint: ['setpoint', 'set', 'target', 'goal']
};

export const CONTROLLER_PARAM_LABELS: Record<string, string> = {
  kp: 'kP',
  ki: 'kI',
  kd: 'kD',
  ks: 'kS',
  kv: 'kV',
  ka: 'kA',
  kg: 'kG',
  izone: 'I Zone',
  setpoint: 'Setpoint'
};

export type ControllerParam = {
  key: string;
  label: string;
  signalId: number;
};

export type ControllerAutoTuneConfig = {
  enabled: boolean;
  modeSignalId: number | null;
  statusSignalId: number | null;
  resultSignalId: number | null;
  startActionSignalId: number | null;
  stopActionSignalId: number | null;
};

export type ControllerWidgetConfig = {
  params: ControllerParam[];
  autotune: ControllerAutoTuneConfig;
};

export function guessControllerParamKey(signal: SignalRow): string | null {
  const leaf = normalizeToken(leafPath(signal.path));
  if (!leaf) return null;

  for (const [key, aliases] of Object.entries(CONTROLLER_PARAM_ALIASES)) {
    const normalizedAliases = aliases.map((alias) => normalizeToken(alias));
    if (normalizedAliases.some((alias) => alias === leaf || leaf.endsWith(alias))) {
      return key;
    }
  }

  return null;
}

export function buildControllerDefaultParams(anchor: SignalRow, signals: SignalRow[]): ControllerParam[] {
  const anchorParent = parentPath(anchor.path);

  const siblings = signals.filter((signal) => {
    if (isNtPublishToggleSignal(signal)) return false;
    if (!isNumericSignal(signal)) return false;
    if (signal.access !== 'write') return false;
    if (signal.signal_id === anchor.signal_id) return true;

    const parent = parentPath(signal.path);
    if (!anchorParent) return parent === '';
    return parent === anchorParent;
  });

  const withKeys = siblings
    .map((signal) => ({
      signal,
      key: guessControllerParamKey(signal)
    }))
    .filter((entry) => entry.key !== null) as Array<{ signal: SignalRow; key: string }>;

  const uniqueByKey = new Map<string, SignalRow>();
  for (const entry of withKeys) {
    if (!uniqueByKey.has(entry.key)) {
      uniqueByKey.set(entry.key, entry.signal);
    }
  }

  const ordered: ControllerParam[] = [];
  for (const key of CONTROLLER_PARAM_ORDER) {
    const signal = uniqueByKey.get(key);
    if (!signal) continue;
    ordered.push({
      key,
      signalId: signal.signal_id,
      label: CONTROLLER_PARAM_LABELS[key] ?? key
    });
  }

  if (ordered.length > 0) {
    return ordered;
  }

  return [
    {
      key: 'value',
      signalId: anchor.signal_id,
      label: leafPath(anchor.path)
    }
  ];
}

export function defaultControllerAutoTune(
  anchor: SignalRow,
  signals: SignalRow[]
): ControllerAutoTuneConfig {
  const anchorParent = parentPath(anchor.path);
  const fallbackParent = anchorParent ? parentPath(anchorParent) : '';

  const startAction =
    findActionSignalByLeaf(
      signals,
      anchorParent,
      ['start_autotune', 'autotune_start', 'start_sysid', 'sysid_start', 'start', 'run']
    ) ??
    (fallbackParent
      ? findActionSignalByLeaf(
          signals,
          fallbackParent,
          ['start_autotune', 'autotune_start', 'start_sysid', 'sysid_start', 'start', 'run']
        )
      : null);
  const stopAction =
    findActionSignalByLeaf(
      signals,
      anchorParent,
      ['stop_autotune', 'autotune_stop', 'stop_sysid', 'sysid_stop', 'stop', 'cancel']
    ) ??
    (fallbackParent
      ? findActionSignalByLeaf(
          signals,
          fallbackParent,
          ['stop_autotune', 'autotune_stop', 'stop_sysid', 'sysid_stop', 'stop', 'cancel']
        )
      : null);

  const modeSignal =
    findSignalByLeaf(
      signals,
      anchorParent,
      ['autotune_mode', 'autotuner_mode', 'sysid_mode', 'mode'],
      ['string', 'i64', 'f64', 'bool']
    ) ??
    (fallbackParent
      ? findSignalByLeaf(
          signals,
          fallbackParent,
          ['autotune_mode', 'autotuner_mode', 'sysid_mode', 'mode'],
          ['string', 'i64', 'f64', 'bool']
        )
      : null);

  const statusSignal =
    findSignalByLeaf(
      signals,
      anchorParent,
      ['autotune_status', 'autotuner_status', 'sysid_status', 'status'],
      ['string', 'bool', 'i64', 'f64']
    ) ??
    (fallbackParent
      ? findSignalByLeaf(
          signals,
          fallbackParent,
          ['autotune_status', 'autotuner_status', 'sysid_status', 'status'],
          ['string', 'bool', 'i64', 'f64']
        )
      : null);

  const resultSignal =
    findSignalByLeaf(
      signals,
      anchorParent,
      ['autotune_result', 'autotuner_result', 'sysid_result', 'result', 'summary'],
      ['string', 'string[]']
    ) ??
    (fallbackParent
      ? findSignalByLeaf(
          signals,
          fallbackParent,
          ['autotune_result', 'autotuner_result', 'sysid_result', 'result', 'summary'],
          ['string', 'string[]']
        )
      : null);

  const enabled =
    startAction !== null ||
    stopAction !== null ||
    modeSignal !== null ||
    statusSignal !== null ||
    resultSignal !== null;

  return {
    enabled,
    modeSignalId: modeSignal?.signal_id ?? null,
    statusSignalId: statusSignal?.signal_id ?? null,
    resultSignalId: resultSignal?.signal_id ?? null,
    startActionSignalId: startAction?.signal_id ?? null,
    stopActionSignalId: stopAction?.signal_id ?? null
  };
}

export function readControllerConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): ControllerWidgetConfig {
  const defaults = defaultControllerAutoTune(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const parsed = Array.isArray(base.params)
    ? base.params
        .map((entry) => {
          if (!isObject(entry)) return null;
          const signalId = toNumber(entry.signalId);
          if (signalId === null || signalId <= 0) return null;
          const label = toString(entry.label, `Signal ${signalId}`).trim() || `Signal ${signalId}`;
          const key = toString(entry.key, normalizeToken(label) || 'value');
          return {
            key,
            label,
            signalId
          };
        })
        .filter((entry): entry is ControllerParam => entry !== null)
    : [];
  const autotuneBase = isObject(base.autotune) ? base.autotune : {};
  const legacyAutotuneBase = isObject(base.autoTune) ? base.autoTune : {};
  const source = Object.keys(autotuneBase).length > 0 ? autotuneBase : legacyAutotuneBase;

  return {
    params: parsed.length > 0 ? parsed : buildControllerDefaultParams(fallbackSignal, signals),
    autotune: {
      enabled: toBoolean(source.enabled, defaults.enabled),
      modeSignalId: resolveSignalIdFromKeys(
        source,
        ['modeSignalId', 'modeId'],
        defaults.modeSignalId,
        signals
      ),
      statusSignalId: resolveSignalIdFromKeys(
        source,
        ['statusSignalId', 'statusId'],
        defaults.statusSignalId,
        signals
      ),
      resultSignalId: resolveSignalIdFromKeys(
        source,
        ['resultSignalId', 'resultId'],
        defaults.resultSignalId,
        signals
      ),
      startActionSignalId: resolveSignalIdFromKeys(
        source,
        ['startActionSignalId', 'startSignalId', 'startActionId'],
        defaults.startActionSignalId,
        signals
      ),
      stopActionSignalId: resolveSignalIdFromKeys(
        source,
        ['stopActionSignalId', 'stopSignalId', 'stopActionId'],
        defaults.stopActionSignalId,
        signals
      )
    }
  };
}
