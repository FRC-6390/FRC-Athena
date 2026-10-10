import { isLayoutWidgetKind, type DashboardWidget, type WidgetLayout } from '../dashboard';
import type { CanvasContext } from './model';
import { DEFAULT_SIGNAL_DROP_SIZE } from './model';
import { absoluteFromGridCell, gridCellFromAbsolute } from './grid-spec';
import { gridSpecOf } from './containers';
import { isAccordionCollapsed, isDescendantOf } from './tree';

/** True when any drawn, visible data widget (not a layout block) overlaps the given band. */
export function areaOverlapsData(
  ctx: CanvasContext,
  xStart: number,
  xEnd: number,
  yStart: number,
  yEnd: number,
  ignoreWidgetId?: string,
  parentLayoutId?: string
): boolean {
  return ctx.widgets.some((widget) => {
    if (widget.id === ignoreWidgetId) return false;
    if (ctx.hidden(widget)) return false;
    if (parentLayoutId !== undefined && widget.parentLayoutId !== parentLayoutId) return false;
    const layout = ctx.layoutOf(widget);
    const widgetYEnd = layout.y + layout.h;
    if (widgetYEnd <= yStart || layout.y >= yEnd) return false;
    const widgetXEnd = layout.x + layout.w;
    return !(widgetXEnd <= xStart || layout.x >= xEnd);
  });
}

/**
 * Where a widget of `dropSize` lands when dropped into (or stacked onto) a layout container. Lists and
 * grids look for the first free slot starting at the requested position; accordions and text blocks have
 * a fixed anchor.
 */
export function resolveLayoutDropPosition(
  ctx: CanvasContext,
  layoutWidget: DashboardWidget,
  fallback: Pick<WidgetLayout, 'x' | 'y'>,
  dropSize: Pick<WidgetLayout, 'w' | 'h'> = DEFAULT_SIGNAL_DROP_SIZE,
  ignoreDataWidgetId?: string
): Pick<WidgetLayout, 'x' | 'y'> {
  const layoutBounds = ctx.boundsOf(layoutWidget);
  const xStart = layoutBounds.x;
  const xEnd = layoutBounds.x + layoutBounds.w;
  const yStart = layoutBounds.y;
  const yEnd = layoutBounds.y + layoutBounds.h;
  const dropW = Math.max(1, Math.min(dropSize.w, xEnd - xStart));
  const dropH = Math.max(1, dropSize.h);
  const fallbackY = Math.max(yStart, Math.min(yEnd - dropH, fallback.y));
  const ignoreId = ignoreDataWidgetId ?? layoutWidget.id;

  if (layoutWidget.kind === 'layout_accordion') {
    return { x: xStart, y: yStart };
  }

  if (layoutWidget.kind === 'layout_list') {
    const listDropW = xEnd - xStart;
    const fallbackRow = Math.floor(fallbackY);
    const maxRow = yEnd - dropH;
    for (let row = fallbackRow; row <= maxRow; row++) {
      if (!areaOverlapsData(ctx, xStart, xStart + listDropW, row, row + dropH, ignoreId, layoutWidget.id)) {
        return { x: xStart, y: row };
      }
    }
    for (let row = Math.floor(yStart); row < fallbackRow; row++) {
      if (!areaOverlapsData(ctx, xStart, xStart + listDropW, row, row + dropH, ignoreId, layoutWidget.id)) {
        return { x: xStart, y: row };
      }
    }
    return { x: xStart, y: yStart };
  }

  if (layoutWidget.kind === 'layout_grid') {
    const spec = gridSpecOf(ctx, layoutWidget);
    if (!spec) {
      return { x: xStart, y: yStart };
    }
    const gridBounds = spec.parent;
    const gridXStart = gridBounds.x;
    const gridYStart = gridBounds.y;
    const gridXEnd = gridBounds.x + gridBounds.w;
    const gridYEnd = gridBounds.y + gridBounds.h;
    const gridDropW = Math.max(1, Math.min(dropW, gridXEnd - gridXStart));
    const gridDropH = Math.max(1, Math.min(dropH, gridYEnd - gridYStart));
    const gridFallbackX = Math.max(gridXStart, Math.min(gridXEnd - gridDropW, fallback.x));
    const gridFallbackY = Math.max(gridYStart, Math.min(gridYEnd - gridDropH, fallback.y));
    const fallbackCellLayout = gridCellFromAbsolute(spec, {
      x: gridFallbackX,
      y: gridFallbackY,
      w: gridDropW,
      h: gridDropH
    });

    const cellW = Math.max(1, Math.min(spec.columns, fallbackCellLayout.w));
    const cellH = Math.max(1, Math.min(spec.rows, fallbackCellLayout.h));
    const maxCellCol = Math.max(1, spec.columns - cellW + 1);
    const maxCellRow = Math.max(1, spec.rows - cellH + 1);
    const startCellCol = Math.max(1, Math.min(maxCellCol, fallbackCellLayout.x));
    const startCellRow = Math.max(1, Math.min(maxCellRow, fallbackCellLayout.y));

    // Scan the grid from the requested cell, wrapping around, for the first free slot.
    for (let rowOffset = 0; rowOffset < maxCellRow; rowOffset++) {
      const row = 1 + ((startCellRow - 1 + rowOffset) % maxCellRow);
      for (let colOffset = 0; colOffset < maxCellCol; colOffset++) {
        const col = 1 + ((startCellCol - 1 + colOffset) % maxCellCol);
        const candidate = absoluteFromGridCell(spec, { x: col, y: row, w: cellW, h: cellH }, ctx.gridColumns);
        if (!areaOverlapsData(ctx, candidate.x, candidate.x + candidate.w, candidate.y, candidate.y + candidate.h, ignoreId, layoutWidget.id)) {
          return { x: candidate.x, y: candidate.y };
        }
      }
    }

    const fallbackAbsolute = absoluteFromGridCell(
      spec,
      { x: startCellCol, y: startCellRow, w: cellW, h: cellH },
      ctx.gridColumns
    );
    return { x: fallbackAbsolute.x, y: fallbackAbsolute.y };
  }

  if (layoutWidget.kind === 'layout_section' || layoutWidget.kind === 'layout_divider' || layoutWidget.kind === 'layout_title') {
    return { x: xStart, y: yStart + 1 };
  }

  return fallback;
}

/** Layout block under a grid cell, preferring the deepest and then the smallest block. */
export function findLayoutWidgetAtCell(
  ctx: CanvasContext,
  cell: Pick<WidgetLayout, 'x' | 'y'>,
  ignoreWidgetId?: string,
  ignoreDescendantsOfId?: string
): DashboardWidget | null {
  const candidates = ctx.widgets.filter((widget) => {
    if (widget.id === ignoreWidgetId) return false;
    if (!isLayoutWidgetKind(widget.kind)) return false;
    if (ignoreDescendantsOfId && isDescendantOf(ctx.byId, widget, ignoreDescendantsOfId)) return false;
    if (ctx.hidden(widget)) return false;
    if (isAccordionCollapsed(widget)) return false;
    const layout = ctx.layoutOf(widget);
    const xEnd = layout.x + layout.w;
    const yEnd = layout.y + layout.h;
    return cell.x >= layout.x && cell.x < xEnd && cell.y >= layout.y && cell.y < yEnd;
  });

  if (candidates.length === 0) return null;

  const depthCache = new Map<string, number>();
  const depthVisiting = new Set<string>();
  const hierarchyDepth = (widgetId: string): number => {
    const cached = depthCache.get(widgetId);
    if (cached !== undefined) return cached;
    if (depthVisiting.has(widgetId)) return 0;
    depthVisiting.add(widgetId);
    const parentId = ctx.byId.get(widgetId)?.parentLayoutId;
    let depth = 0;
    if (parentId && ctx.byId.has(parentId)) {
      depth = hierarchyDepth(parentId) + 1;
    }
    depthVisiting.delete(widgetId);
    depthCache.set(widgetId, depth);
    return depth;
  };

  candidates.sort((a, b) => {
    const depthDiff = hierarchyDepth(b.id) - hierarchyDepth(a.id);
    if (depthDiff !== 0) return depthDiff;
    const layoutA = ctx.layoutOf(a);
    const layoutB = ctx.layoutOf(b);
    const areaA = layoutA.w * layoutA.h;
    const areaB = layoutB.w * layoutB.h;
    if (areaA !== areaB) return areaA - areaB;
    return layoutB.y - layoutA.y || layoutB.x - layoutA.x || a.id.localeCompare(b.id);
  });

  return candidates[0] ?? null;
}
