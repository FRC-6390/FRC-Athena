import type { DashboardWidget, WidgetConfigRecord, WidgetLayout } from '../dashboard';

/** Compact widget builder for canvas tests. */
export function widget(
  id: string,
  kind: DashboardWidget['kind'],
  layout: WidgetLayout,
  extra: { parentLayoutId?: string; config?: WidgetConfigRecord; signalId?: number } = {}
): DashboardWidget {
  return {
    id,
    signalId: extra.signalId ?? 0,
    kind,
    title: id,
    layout,
    parentLayoutId: extra.parentLayoutId,
    config: extra.config
  };
}

export const at = (x: number, y: number, w: number, h: number): WidgetLayout => ({ x, y, w, h });
