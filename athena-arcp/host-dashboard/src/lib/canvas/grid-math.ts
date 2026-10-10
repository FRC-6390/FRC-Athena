import { DEFAULT_GRID_COLUMNS, type DashboardWidget, type WidgetLayout } from '../dashboard';
import {
  GRID_GAP,
  MAX_GRID_COLUMNS,
  MIN_CELL_SIZE,
  MIN_GRID_COLUMNS,
  MIN_ROWS,
  SCROLLBAR_SAFETY_GUTTER
} from './model';

/** Columns needed to show every widget, never fewer than the default and never more than the maximum. */
export function gridColumnsFor(widgets: readonly DashboardWidget[]): number {
  let maxColumn = DEFAULT_GRID_COLUMNS;
  for (const widget of widgets) {
    const right = widget.layout.x + widget.layout.w;
    if (right > maxColumn) maxColumn = right;
  }
  const resolved = Math.ceil(Math.max(DEFAULT_GRID_COLUMNS, maxColumn));
  return Math.max(MIN_GRID_COLUMNS, Math.min(MAX_GRID_COLUMNS, resolved));
}

export function cellSizeFor(gridWidth: number, gridColumns: number): number {
  return Math.max(MIN_CELL_SIZE, (gridWidth - (gridColumns - 1) * GRID_GAP) / gridColumns);
}

export function visibleRowsFor(gridHeight: number, cellSize: number): number {
  return Math.max(1, Math.floor((gridHeight + GRID_GAP) / (cellSize + GRID_GAP)));
}

export function canvasRowsFor(widgets: readonly DashboardWidget[], visibleRows: number): number {
  let maxRow = MIN_ROWS;
  for (const widget of widgets) {
    const bottom = widget.layout.y + widget.layout.h;
    if (bottom > maxRow) maxRow = bottom;
  }
  return Math.ceil(Math.max(maxRow, visibleRows));
}

export function contentSizeFor(gridColumns: number, canvasRows: number, cellSize: number): { width: number; height: number } {
  return {
    width: gridColumns * cellSize + (gridColumns - 1) * GRID_GAP,
    height: canvasRows * cellSize + (canvasRows - 1) * GRID_GAP
  };
}

export type PixelRect = { left: number; top: number; width: number; height: number };

/**
 * Pixel box of a grid-unit layout. Widgets inside a container drop the trailing gap so they fill their cell
 * exactly; top-level widgets keep it so neighbours do not touch.
 */
export function pixelRectFor(layout: WidgetLayout, cellSize: number, withoutTrailingGap: boolean): PixelRect {
  const step = cellSize + GRID_GAP;
  const left = (layout.x - 1) * step;
  const top = (layout.y - 1) * step;
  const width = withoutTrailingGap
    ? layout.w * step
    : layout.w >= 1
      ? layout.w * step - GRID_GAP
      : layout.w * step;
  const height = withoutTrailingGap
    ? layout.h * step
    : layout.h >= 1
      ? layout.h * step - GRID_GAP
      : layout.h * step;
  return { left, top, width, height };
}

/** Converts a viewport point to fractional grid units (1-based), or null when the grid is not measured. */
export function pointToGridUnits(
  clientX: number,
  clientY: number,
  surface: { left: number; top: number; scrollLeft: number; scrollTop: number },
  cellSize: number
): { x: number; y: number } | null {
  const step = cellSize + GRID_GAP;
  if (step <= 0) return null;
  return {
    x: (clientX - surface.left + surface.scrollLeft) / step + 1,
    y: (clientY - surface.top + surface.scrollTop) / step + 1
  };
}

export function layoutsEqual(a: WidgetLayout, b: WidgetLayout): boolean {
  return a.x === b.x && a.y === b.y && a.w === b.w && a.h === b.h;
}

export function layoutContainsPoint(layout: WidgetLayout, x: number, y: number): boolean {
  return x >= layout.x && x < layout.x + layout.w && y >= layout.y && y < layout.y + layout.h;
}

export type RenderedExtent = { maxRightPx: number; maxBottomPx: number };

/** Largest pixel extent of the drawn widgets; the scroll area grows to fit them. */
export function renderedExtent(
  entries: ReadonlyArray<{ layout: WidgetLayout; withoutTrailingGap: boolean }>,
  cellSize: number
): RenderedExtent {
  let maxRightPx = 0;
  let maxBottomPx = 0;
  for (const entry of entries) {
    const rect = pixelRectFor(entry.layout, cellSize, entry.withoutTrailingGap);
    const right = rect.left + Math.max(1, rect.width);
    const bottom = rect.top + Math.max(1, rect.height);
    if (right > maxRightPx) maxRightPx = right;
    if (bottom > maxBottomPx) maxBottomPx = bottom;
  }
  return { maxRightPx, maxBottomPx };
}

export type ScrollMetrics = {
  effectiveWidth: number;
  effectiveHeight: number;
  overflowX: boolean;
  overflowY: boolean;
  scrollWidth: number;
  scrollHeight: number;
};

/** Spacer size: content or drawn extent, plus a scrollbar gutter on the opposite axis when overflowing. */
export function scrollMetricsFor(
  content: { width: number; height: number },
  rendered: RenderedExtent,
  gridWidth: number,
  gridHeight: number
): ScrollMetrics {
  const effectiveWidth = Math.max(content.width, Math.ceil(rendered.maxRightPx));
  const effectiveHeight = Math.max(content.height, Math.ceil(rendered.maxBottomPx));
  const overflowX = effectiveWidth > gridWidth;
  const overflowY = effectiveHeight > gridHeight;
  return {
    effectiveWidth,
    effectiveHeight,
    overflowX,
    overflowY,
    scrollWidth: effectiveWidth + (overflowY ? SCROLLBAR_SAFETY_GUTTER : 0),
    scrollHeight: effectiveHeight + (overflowX ? SCROLLBAR_SAFETY_GUTTER : 0)
  };
}

/** Inline style for a card at a fixed box, with an optional stacking index. */
export function boxStyle(box: PixelRect, zIndex?: number, extra = ''): string {
  const z = zIndex !== undefined ? `z-index:${zIndex};` : '';
  return `transform:translate3d(${box.left}px,${box.top}px,0);width:${Math.max(1, box.width)}px;height:${Math.max(1, box.height)}px;${z}${extra}`;
}
