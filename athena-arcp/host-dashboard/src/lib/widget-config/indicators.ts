import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { clamp01, isObject, normalizeSignalId, parentPath, toBoolean, toNumber, toString } from './common';

export type BarWidgetMode = 'progress' | 'usage';

export type BarWidgetConfig = {
  min: number;
  max: number;
  unit: string;
  mode: BarWidgetMode;
  warn: number;
  crit: number;
};

export type DialWidgetConfig = {
  min: number;
  max: number;
  unit: string;
};

export type TimerWidgetMode = 'elapsed' | 'countdown';

export type TimerWidgetConfig = {
  mode: TimerWidgetMode;
  durationSec: number;
};

export type StatusMatrixHealth = 'true' | 'false';

export type StatusMatrixMode = 'status' | 'toggle' | 'toggle_value';

export type StatusMatrixItem = {
  signalId: number;
  label: string;
  group: string;
  healthyWhen: StatusMatrixHealth;
  toggleSignalId: number | null;
  valueSignalId: number | null;
};

export type StatusMatrixConfig = {
  items: StatusMatrixItem[];
  columns: number;
  showSummary: boolean;
  mode: StatusMatrixMode;
};

export function readBarConfig(raw: WidgetConfigRecord | undefined): BarWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const min = toNumber(base.min) ?? 0;
  const maxRaw = toNumber(base.max) ?? 1;
  const max = maxRaw <= min ? min + 1 : maxRaw;
  const modeRaw = toString(base.mode, 'progress').toLowerCase();
  const mode: BarWidgetMode = modeRaw === 'usage' ? 'usage' : 'progress';

  const warnRaw = toNumber(base.warn) ?? 0.7;
  const critRaw = toNumber(base.crit) ?? 0.9;
  const warn = clamp01(warnRaw);
  const crit = clamp01(Math.max(warn, critRaw));

  return {
    min,
    max,
    mode,
    warn,
    crit,
    unit: toString(base.unit, '')
  };
}

export function readDialConfig(raw: WidgetConfigRecord | undefined): DialWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const min = toNumber(base.min) ?? 0;
  const maxRaw = toNumber(base.max) ?? 100;
  return {
    min,
    max: maxRaw <= min ? min + 1 : maxRaw,
    unit: toString(base.unit, '')
  };
}

export function readTimerConfig(raw: WidgetConfigRecord | undefined): TimerWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const modeRaw = toString(base.mode, 'countdown').toLowerCase();
  const mode: TimerWidgetMode = modeRaw === 'elapsed' ? 'elapsed' : 'countdown';
  return {
    mode,
    durationSec: Math.max(1, toNumber(base.durationSec) ?? 150)
  };
}

export function defaultStatusMatrixItems(anchor: SignalRow, signals: SignalRow[]): StatusMatrixItem[] {
  const anchorParent = parentPath(anchor.path);
  const siblings = signals
    .filter((signal) => {
      if (signal.signal_type !== 'bool') return false;
      if (signal.signal_id === anchor.signal_id) return true;
      return parentPath(signal.path) === anchorParent;
    })
    .sort((a, b) => a.path.localeCompare(b.path))
    .slice(0, 96);

  if (siblings.length === 0 && anchor.signal_type === 'bool') {
    siblings.push(anchor);
  }

  const fallbackGroup = anchorParent || 'status';
  return siblings.map((signal) => ({
    signalId: signal.signal_id,
    label: leafPath(signal.path),
    group: fallbackGroup,
    healthyWhen: 'true',
    toggleSignalId: signal.access === 'write' ? signal.signal_id : null,
    valueSignalId: null
  }));
}

export function parseStatusMatrixItem(value: unknown): StatusMatrixItem | null {
  if (!isObject(value)) return null;
  const signalId = normalizeSignalId(value.signalId);
  if (signalId === null) return null;

  const healthyWhenRaw = toString(value.healthyWhen, 'true').toLowerCase();
  const healthyWhen: StatusMatrixHealth = healthyWhenRaw === 'false' ? 'false' : 'true';

  return {
    signalId,
    label: toString(value.label, `Signal ${signalId}`).trim() || `Signal ${signalId}`,
    group: toString(value.group, 'status').trim() || 'status',
    healthyWhen,
    toggleSignalId: normalizeSignalId(value.toggleSignalId),
    valueSignalId: normalizeSignalId(value.valueSignalId)
  };
}

export function readStatusMatrixConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): StatusMatrixConfig {
  const base = isObject(raw) ? raw : {};
  const items = Array.isArray(base.items)
    ? base.items
        .map((entry) => parseStatusMatrixItem(entry))
        .filter((entry): entry is StatusMatrixItem => entry !== null)
    : [];

  const modeRaw = toString(base.mode, 'status').trim().toLowerCase();
  const mode: StatusMatrixMode =
    modeRaw === 'toggle' || modeRaw === 'toggle_value' ? modeRaw : 'status';

  return {
    items: items.length > 0 ? items : defaultStatusMatrixItems(fallbackSignal, signals),
    columns: Math.max(1, Math.min(8, Math.floor(toNumber(base.columns) ?? 4))),
    showSummary: toBoolean(base.showSummary, true),
    mode
  };
}

export function addStatusMatrixSignal(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[],
  signal: SignalRow
): StatusMatrixConfig {
  const config = readStatusMatrixConfig(raw, fallbackSignal, signals);
  if (config.items.some((item) => item.signalId === signal.signal_id)) {
    return config;
  }

  return {
    ...config,
    items: [
      ...config.items,
      {
        signalId: signal.signal_id,
        label: leafPath(signal.path),
        group: parentPath(signal.path) || 'status',
        healthyWhen: 'true',
        toggleSignalId: signal.access === 'write' ? signal.signal_id : null,
        valueSignalId: null
      }
    ]
  };
}
