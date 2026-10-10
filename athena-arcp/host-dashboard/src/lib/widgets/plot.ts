import type { WidgetModule } from './contract';

export type PlotWidgetConfig = {
  /** Visible history window in seconds. */
  windowSeconds: number;
  yMin: number | null;
  yMax: number | null;
};

export const PLOT_WINDOW_LIMITS = { min: 2, max: 300, fallback: 20 } as const;

/** Reads a saved plot config, falling back to defaults for anything missing or malformed. */
export function readPlotConfig(raw: Record<string, unknown> | undefined): PlotWidgetConfig {
  const base = raw ?? {};
  const window = typeof base.windowSeconds === 'number' && Number.isFinite(base.windowSeconds)
    ? Math.min(PLOT_WINDOW_LIMITS.max, Math.max(PLOT_WINDOW_LIMITS.min, base.windowSeconds))
    : PLOT_WINDOW_LIMITS.fallback;
  const bound = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
  return { windowSeconds: window, yMin: bound(base.yMin), yMax: bound(base.yMax) };
}

/** New-contract plot: a scrolling trace of any numeric signal (targets, references, measurements). */
export const PLOT_MODULE: WidgetModule<PlotWidgetConfig> = {
  id: 'plot',
  title: 'Plot',
  summary: 'Scrolling trace of a numeric signal with a configurable window.',
  accepts: ['numeric'],
  access: 'any',
  excludeRoles: ['action'],
  legacy: false,
  readConfig: readPlotConfig
};
