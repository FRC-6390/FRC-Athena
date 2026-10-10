import type { SignalRow } from '../arcp';
import type { ControllerParam, ControllerWidgetConfig } from './controller';
import type { GraphSeriesStyle, GraphWidgetConfig } from './graph';

/*
 * Pure helpers behind the widget config editor. The editor sections call these to build the next config;
 * they never touch component state, so each rule can be tested on its own.
 */

/** Last segment of a signal path. */
export function leafPath(path: string): string {
  const parts = path.split('/').filter(Boolean);
  return parts[parts.length - 1] ?? path;
}

/** Lower-case alphanumeric form of a name, for loose matching (`kP`, `k_p`, `K-P` all match). */
export function normalizeToken(value: string): string {
  return value.toLowerCase().replace(/[^a-z0-9]+/g, '');
}

export function isNumeric(signal: SignalRow | null | undefined): signal is SignalRow {
  if (!signal) return false;
  return signal.signal_type === 'f64' || signal.signal_type === 'i64';
}

export function isWritableNumeric(signal: SignalRow | null | undefined): signal is SignalRow {
  return !!signal && isNumeric(signal) && signal.access === 'write';
}

/** Parameter key: lower-case words joined by `_`, never empty. */
export function normalizeControllerKey(raw: string): string {
  const normalized = raw
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '_')
    .replace(/^_+|_+$/g, '');
  return normalized || 'value';
}

/** A key not used by any existing parameter, suffixed `_2`, `_3`, ... when needed. */
export function nextControllerKey(existing: ControllerParam[], preferred: string): string {
  const base = normalizeControllerKey(preferred);
  const used = new Set(existing.map((entry) => normalizeControllerKey(entry.key)));
  if (!used.has(base)) return base;
  let index = 2;
  while (used.has(`${base}_${index}`)) {
    index += 1;
  }
  return `${base}_${index}`;
}

/** Edits one controller parameter. */
export function patchControllerParam(
  controller: ControllerWidgetConfig,
  index: number,
  patch: Partial<ControllerParam>
): ControllerWidgetConfig {
  return {
    ...controller,
    params: controller.params.map((entry, entryIndex) => (entryIndex === index ? { ...entry, ...patch } : entry))
  };
}

/** Removes a parameter, keeping at least one. Returns null when nothing may be removed. */
export function removeControllerParamAt(controller: ControllerWidgetConfig, index: number): ControllerWidgetConfig | null {
  if (controller.params.length <= 1) return null;
  return {
    ...controller,
    params: controller.params.filter((_, entryIndex) => entryIndex !== index)
  };
}

/**
 * Appends a parameter for a signal. Returns null when that signal is already a parameter. The caller checks
 * that the signal is a writable number.
 */
export function appendControllerParam(
  controller: ControllerWidgetConfig,
  signal: Pick<SignalRow, 'signal_id' | 'path'>
): ControllerWidgetConfig | null {
  if (controller.params.some((entry) => entry.signalId === signal.signal_id)) return null;
  const label = leafPath(signal.path);
  return {
    ...controller,
    params: [
      ...controller.params,
      { key: nextControllerKey(controller.params, label), label, signalId: signal.signal_id }
    ]
  };
}

/** Sets the draw style of one graph series. */
export function withSeriesStyle(graph: GraphWidgetConfig, signalId: number, style: GraphSeriesStyle): GraphWidgetConfig {
  return {
    ...graph,
    series: graph.series.map((series) => (series.signalId === signalId ? { ...series, style } : series))
  };
}

/** Sets the colour of one graph series. */
export function withSeriesColor(graph: GraphWidgetConfig, signalId: number, color: string): GraphWidgetConfig {
  return {
    ...graph,
    series: graph.series.map((series) => (series.signalId === signalId ? { ...series, color } : series))
  };
}

/** Removes a graph series, keeping at least one. Returns null when nothing may be removed. */
export function withoutSeries(graph: GraphWidgetConfig, signalId: number): GraphWidgetConfig | null {
  if (graph.series.length <= 1) return null;
  return { ...graph, series: graph.series.filter((series) => series.signalId !== signalId) };
}

/** Number typed into an optional axis limit: blank or non-finite means no limit. */
export function parseOptionalNumber(raw: string): number | null {
  if (raw.trim() === '') return null;
  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}
