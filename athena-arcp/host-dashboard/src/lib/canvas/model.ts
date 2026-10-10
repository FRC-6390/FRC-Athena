import {
  clampLayout,
  DEFAULT_GRID_COLUMNS,
  type DashboardWidget,
  type WidgetLayout
} from '../dashboard';
import type { DragState } from './drag';
import { normalizedContainerLayouts } from './containers';
import { hiddenByCollapsedAccordion } from './tree';

/** Pixel and grid constants shared by the classic canvas. */
export const GRID_GAP = 6;
export const MIN_ROWS = 8;
export const MIN_GRID_COLUMNS = DEFAULT_GRID_COLUMNS;
export const MAX_GRID_COLUMNS = 96;
export const MIN_CELL_SIZE = 22;
export const SCROLLBAR_SAFETY_GUTTER = 18;
export const DEFAULT_SIGNAL_DROP_SIZE: Pick<WidgetLayout, 'w' | 'h'> = { w: 2, h: 1 };

export type DragMode = 'move' | 'resize';

/**
 * Everything the layout math needs to know about the canvas at one moment. The component rebuilds it when
 * widgets, the drag or the measured bounds change; the functions never read component state directly.
 */
export type CanvasContext = {
  widgets: readonly DashboardWidget[];
  byId: ReadonlyMap<string, DashboardWidget>;
  gridColumns: number;
  /** Clamped stored layout, replaced by the live drag layout while that widget is dragged. */
  dragLayoutOf: (widget: DashboardWidget) => WidgetLayout;
  /** Layout as drawn: container children normalised into their parent, drag applied. */
  layoutOf: (widget: DashboardWidget) => WidgetLayout;
  /** Inner content bounds of a layout container (frozen while a drag is in progress). */
  boundsOf: (widget: DashboardWidget) => WidgetLayout;
  /** True when the widget sits inside a collapsed accordion and is not drawn. */
  hidden: (widget: DashboardWidget) => boolean;
};

/**
 * Builds the context for one snapshot. `measuredBounds` supplies content bounds measured from the DOM for
 * layout containers; without it a container's content bounds are its stored layout.
 */
export function buildCanvasContext(input: {
  widgets: readonly DashboardWidget[];
  gridColumns: number;
  drag: DragState | null;
  measuredBounds?: (widget: DashboardWidget) => WidgetLayout;
}): CanvasContext {
  const { widgets, gridColumns, drag } = input;
  const byId = new Map(widgets.map((widget) => [widget.id, widget] as const));
  const dragLayoutOf = (widget: DashboardWidget): WidgetLayout =>
    drag && drag.widgetId === widget.id ? drag.currentLayout : clampLayout(widget.layout, gridColumns);
  const boundsOf = (widget: DashboardWidget): WidgetLayout => {
    const frozen = drag?.frozenLayoutBounds.get(widget.id);
    if (frozen) return frozen;
    return input.measuredBounds ? input.measuredBounds(widget) : clampLayout(widget.layout, gridColumns);
  };
  const hiddenById = hiddenByCollapsedAccordion(widgets, byId);
  const normalized = normalizedContainerLayouts({ widgets, byId, gridColumns, dragLayoutOf, boundsOf });
  const layoutOf = (widget: DashboardWidget): WidgetLayout => {
    if (drag && drag.widgetId === widget.id) return drag.currentLayout;
    return normalized.get(widget.id) ?? clampLayout(widget.layout, gridColumns);
  };
  return {
    widgets,
    byId,
    gridColumns,
    dragLayoutOf,
    layoutOf,
    boundsOf,
    hidden: (widget) => hiddenById.get(widget.id) ?? false
  };
}
