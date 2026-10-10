import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { isObject, signalRole, toBoolean, toNumber, toString } from './common';

export const GRAPH_COLORS = ['#f43f5e', '#38bdf8', '#34d399', '#f59e0b', '#a78bfa', '#f97316', '#22d3ee'];

export type GraphSeriesStyle = 'line' | 'step' | 'dot';

export type GraphSeries = {
  signalId: number;
  label: string;
  color: string;
  style: GraphSeriesStyle;
  role: string;
};

export type GraphWidgetConfig = {
  series: GraphSeries[];
  yMin: number | null;
  yMax: number | null;
  showLegend: boolean;
};

export function graphColor(index: number): string {
  return GRAPH_COLORS[index % GRAPH_COLORS.length] ?? GRAPH_COLORS[0];
}

export function graphSeriesFromSignal(signal: SignalRow, index: number): GraphSeries {
  return {
    signalId: signal.signal_id,
    label: leafPath(signal.path),
    color: graphColor(index),
    style: 'line',
    role: signalRole(signal)
  };
}

export function parseGraphSeries(value: unknown, index: number): GraphSeries | null {
  if (!isObject(value)) return null;
  const signalId = toNumber(value.signalId);
  if (signalId === null || signalId <= 0) return null;

  const styleRaw = toString(value.style, 'line').toLowerCase();
  const style: GraphSeriesStyle =
    styleRaw === 'step' || styleRaw === 'dot' ? (styleRaw as GraphSeriesStyle) : 'line';

  const color = toString(value.color, graphColor(index)).trim() || graphColor(index);
  const label = toString(value.label, `signal ${signalId}`).trim() || `signal ${signalId}`;
  const role = toString(value.role, 'metric');

  return {
    signalId,
    style,
    color,
    label,
    role
  };
}

export function readGraphConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow
): GraphWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const items = Array.isArray(base.series)
    ? base.series
        .map((entry, index) => parseGraphSeries(entry, index))
        .filter((entry): entry is GraphSeries => entry !== null)
    : [];

  return {
    series: items.length > 0 ? items : [graphSeriesFromSignal(fallbackSignal, 0)],
    yMin: toNumber(base.yMin),
    yMax: toNumber(base.yMax),
    showLegend: toBoolean(base.showLegend, true)
  };
}

export function addGraphSeries(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  newSignal: SignalRow
): GraphWidgetConfig {
  const config = readGraphConfig(raw, fallbackSignal);
  if (config.series.some((series) => series.signalId === newSignal.signal_id)) {
    return config;
  }

  return {
    ...config,
    series: [...config.series, graphSeriesFromSignal(newSignal, config.series.length)]
  };
}
