import { clampLayout, isLayoutWidgetKind, type DashboardWidget, type WidgetLayout } from '../../lib/dashboard';
import { GRID_GAP } from '../../lib/canvas/model';
import { normalizeContainerAxisForBounds, type LayoutSource } from '../../lib/canvas/containers';
import { isContainerLayoutKind, renderOrder } from '../../lib/canvas/tree';

/**
 * DOM measurement of layout blocks. Containers are measured from their rendered content box, then anchored
 * to their parent exactly as the model would place them. Results are grid units.
 */

function escapedWidgetId(id: string): string {
  if (typeof CSS !== 'undefined' && typeof CSS.escape === 'function') {
    return CSS.escape(id);
  }
  return id.replace(/"/g, '\\"');
}

/** The element whose box defines a layout block's content area. */
function contentElementOf(gridEl: HTMLElement, layoutWidget: DashboardWidget): HTMLElement | null {
  const rootEl = gridEl.querySelector(`.widget-card[data-widget-id="${escapedWidgetId(layoutWidget.id)}"]`) as HTMLElement | null;
  if (!rootEl) return null;
  if (layoutWidget.kind === 'layout_grid') {
    return (rootEl.querySelector('.layout-grid') as HTMLElement | null) ?? rootEl;
  }
  if (layoutWidget.kind === 'layout_list') {
    return (rootEl.querySelector('.layout-list') as HTMLElement | null) ?? rootEl;
  }
  if (layoutWidget.kind === 'layout_accordion') {
    return (rootEl.querySelector('.layout-accordion-body') as HTMLElement | null) ?? rootEl;
  }
  return (rootEl.querySelector('.widget-body') as HTMLElement | null) ?? rootEl;
}

/** Inner box of one layout block in grid units, or null when it is not rendered. */
function measureOne(gridEl: HTMLElement, widget: DashboardWidget, cellSize: number, gridColumns: number): WidgetLayout | null {
  const contentEl = contentElementOf(gridEl, widget);
  if (!contentEl) return null;

  const gridRect = gridEl.getBoundingClientRect();
  const contentRect = contentEl.getBoundingClientRect();
  const step = cellSize + GRID_GAP;
  const computedStyle = window.getComputedStyle(contentEl);
  const borderLeft = Number.parseFloat(computedStyle.borderLeftWidth) || 0;
  const borderTop = Number.parseFloat(computedStyle.borderTopWidth) || 0;
  const borderRight = Number.parseFloat(computedStyle.borderRightWidth) || 0;
  const borderBottom = Number.parseFloat(computedStyle.borderBottomWidth) || 0;
  const innerLeftPx = contentRect.left - gridRect.left + gridEl.scrollLeft + borderLeft;
  const innerTopPx = contentRect.top - gridRect.top + gridEl.scrollTop + borderTop;
  const innerWidthPx = Math.max(0, contentRect.width - borderLeft - borderRight);
  const innerHeightPx = Math.max(0, contentRect.height - borderTop - borderBottom);
  return clampLayout(
    {
      x: innerLeftPx / step + 1,
      y: innerTopPx / step + 1,
      w: Math.max(1e-3, innerWidthPx / step),
      h: Math.max(1e-3, innerHeightPx / step)
    },
    gridColumns
  );
}

/**
 * Content bounds of every layout block, parents before children. Blocks that are not rendered (or no grid
 * element yet) fall back to their stored layout.
 */
export function measureLayoutBounds(
  gridEl: HTMLElement | null,
  widgets: readonly DashboardWidget[],
  cellSize: number,
  gridColumns: number
): Map<string, WidgetLayout> {
  const map = new Map<string, WidgetLayout>();
  const byId = new Map(widgets.map((widget) => [widget.id, widget] as const));
  const source: LayoutSource = {
    widgets,
    byId,
    gridColumns,
    dragLayoutOf: (widget) => clampLayout(widget.layout, gridColumns),
    boundsOf: (widget) => map.get(widget.id) ?? clampLayout(widget.layout, gridColumns)
  };
  for (const widget of renderOrder(widgets, byId)) {
    if (!isLayoutWidgetKind(widget.kind)) continue;
    const fallback = clampLayout(widget.layout, gridColumns);
    const measured = gridEl ? measureOne(gridEl, widget, cellSize, gridColumns) : null;
    if (!measured) {
      map.set(widget.id, fallback);
    } else if (isContainerLayoutKind(widget.kind)) {
      const anchor = normalizeContainerAxisForBounds(source, widget, fallback);
      map.set(widget.id, clampLayout({ ...measured, x: anchor.x, w: anchor.w }, gridColumns));
    } else {
      map.set(widget.id, measured);
    }
  }
  return map;
}
