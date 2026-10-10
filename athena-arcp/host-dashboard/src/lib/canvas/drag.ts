import { clampLayout, isLayoutWidgetKind, layoutsOverlap, type DashboardWidget, type WidgetLayout } from '../dashboard';
import { GRID_GAP, type CanvasContext, type DragMode } from './model';
import { absoluteFromGridCell, gridCellFromAbsolute } from './grid-spec';
import { gridSpecOf } from './containers';
import { layoutContainsPoint, layoutsEqual } from './grid-math';
import { snapLayoutToCanvasCells, snapMoveWithinContainer } from './snap';
import { findContainerParent, isAccordionCollapsed } from './tree';
import { resolveLayoutDropPosition } from './placement';

export type DragState = {
  widgetId: string;
  mode: DragMode;
  startX: number;
  startY: number;
  pointerX: number;
  pointerY: number;
  startGridX: number;
  startGridY: number;
  anchorOffsetX: number;
  anchorOffsetY: number;
  resizeOffsetX: number;
  resizeOffsetY: number;
  frozenLayoutBounds: Map<string, WidgetLayout>;
  startLayout: WidgetLayout;
  currentLayout: WidgetLayout;
};

/**
 * Widget that a gesture acts on. Resizing a child of an expanded accordion resizes the accordion shell.
 */
export function dragTargetFor(ctx: CanvasContext, widget: DashboardWidget, mode: DragMode): DashboardWidget {
  if (mode !== 'resize') return widget;
  const parent = findContainerParent(ctx.byId, widget);
  if (parent?.kind === 'layout_accordion' && !isAccordionCollapsed(parent)) return parent;
  return widget;
}

/** Drag state at pointer-down. `startLayout` must already be the drawn layout of the target. */
export function beginDragState(input: {
  target: DashboardWidget;
  mode: DragMode;
  clientX: number;
  clientY: number;
  pointerOnCanvas: { x: number; y: number } | null;
  startLayout: WidgetLayout;
  frozenLayoutBounds: Map<string, WidgetLayout>;
}): DragState {
  const { startLayout: clampedStart, pointerOnCanvas } = input;
  const startGridX = pointerOnCanvas?.x ?? clampedStart.x;
  const startGridY = pointerOnCanvas?.y ?? clampedStart.y;
  return {
    widgetId: input.target.id,
    mode: input.mode,
    startX: input.clientX,
    startY: input.clientY,
    pointerX: input.clientX,
    pointerY: input.clientY,
    startGridX,
    startGridY,
    anchorOffsetX: startGridX - clampedStart.x,
    anchorOffsetY: startGridY - clampedStart.y,
    resizeOffsetX: clampedStart.x + clampedStart.w - startGridX,
    resizeOffsetY: clampedStart.y + clampedStart.h - startGridY,
    frozenLayoutBounds: input.frozenLayoutBounds,
    startLayout: clampedStart,
    currentLayout: clampedStart
  };
}

/** True when a data widget candidate would overlap another data widget. Layout blocks never collide. */
export function hasCollision(ctx: CanvasContext, widgetId: string, candidate: WidgetLayout): boolean {
  const movingWidget = ctx.byId.get(widgetId);
  if (!movingWidget || isLayoutWidgetKind(movingWidget.kind) || ctx.hidden(movingWidget)) return false;
  return ctx.widgets.some((widget) => {
    if (widget.id === widgetId) return false;
    if (ctx.hidden(widget)) return false;
    if (isLayoutWidgetKind(widget.kind)) return false;
    return layoutsOverlap(ctx.layoutOf(widget), candidate);
  });
}

/**
 * Layout the dragged widget should take for the pointer position, given its container. Pointer coordinates
 * are in canvas grid units (`pointerOnCanvas`) when the grid is measured, otherwise derived from pixels.
 */
function dragCandidate(
  ctx: CanvasContext,
  drag: DragState,
  moving: DashboardWidget,
  pointerX: number,
  pointerY: number,
  pointerOnCanvas: { x: number; y: number } | null,
  cellSize: number
): WidgetLayout {
  const baseUnitsPerCell = cellSize + GRID_GAP;
  const pointerGridX =
    pointerOnCanvas?.x ?? drag.startGridX + (pointerX - drag.startX) / Math.max(1e-6, baseUnitsPerCell);
  const pointerGridY =
    pointerOnCanvas?.y ?? drag.startGridY + (pointerY - drag.startY) / Math.max(1e-6, baseUnitsPerCell);
  const freeMoveCandidate: WidgetLayout = {
    ...drag.startLayout,
    x: pointerGridX - drag.anchorOffsetX,
    y: pointerGridY - drag.anchorOffsetY
  };
  let freeResizeCandidate: WidgetLayout = {
    ...drag.startLayout,
    w: Math.max(1e-3, pointerGridX + drag.resizeOffsetX - drag.startLayout.x),
    h: Math.max(1e-3, pointerGridY + drag.resizeOffsetY - drag.startLayout.y)
  };
  if (drag.mode === 'resize' && moving.kind === 'layout_accordion' && isAccordionCollapsed(moving)) {
    // Collapsed accordions stay one row tall; resize gestures should affect width.
    freeResizeCandidate = { ...freeResizeCandidate, h: Math.max(1, Math.round(drag.startLayout.h)) };
  }

  const cols = ctx.gridColumns;
  const containerParent = findContainerParent(ctx.byId, moving);
  const toBounds = (parentLayout: WidgetLayout): WidgetLayout =>
    clampLayout({ ...drag.startLayout, x: parentLayout.x, y: parentLayout.y, w: parentLayout.w, h: parentLayout.h }, cols);

  if (containerParent?.kind === 'layout_grid') {
    const spec = gridSpecOf(ctx, containerParent);
    if (!spec) return drag.mode === 'move' ? freeMoveCandidate : freeResizeCandidate;
    const startCell = gridCellFromAbsolute(spec, drag.startLayout);
    if (drag.mode === 'move') {
      const centerX = freeMoveCandidate.x + freeMoveCandidate.w * 0.5;
      const centerY = freeMoveCandidate.y + freeMoveCandidate.h * 0.5;
      if (!layoutContainsPoint(spec.parent, centerX, centerY)) return freeMoveCandidate;
      const candidateCell = gridCellFromAbsolute(spec, freeMoveCandidate);
      const maxCol = Math.max(1, spec.columns - startCell.w + 1);
      const maxRow = Math.max(1, spec.rows - startCell.h + 1);
      const targetCell: WidgetLayout = {
        x: Math.max(1, Math.min(maxCol, candidateCell.x)),
        y: Math.max(1, Math.min(maxRow, candidateCell.y)),
        w: startCell.w,
        h: startCell.h
      };
      return absoluteFromGridCell(spec, targetCell, cols) ?? drag.startLayout;
    }
    const candidateCell = gridCellFromAbsolute(spec, freeResizeCandidate);
    const maxW = Math.max(1, spec.columns - startCell.x + 1);
    const maxH = Math.max(1, spec.rows - startCell.y + 1);
    const targetCell: WidgetLayout = {
      x: startCell.x,
      y: startCell.y,
      w: Math.max(1, Math.min(maxW, candidateCell.w)),
      h: Math.max(1, Math.min(maxH, candidateCell.h))
    };
    return absoluteFromGridCell(spec, targetCell, cols) ?? drag.startLayout;
  }

  if (containerParent?.kind === 'layout_accordion') {
    const parentLayout = ctx.boundsOf(containerParent);
    if (drag.mode === 'move') {
      const centerX = freeMoveCandidate.x + freeMoveCandidate.w * 0.5;
      const centerY = freeMoveCandidate.y + freeMoveCandidate.h * 0.5;
      if (!layoutContainsPoint(parentLayout, centerX, centerY)) return freeMoveCandidate;
    }
    return toBounds(parentLayout);
  }

  if (containerParent?.kind === 'layout_list') {
    const parentLayout = ctx.boundsOf(containerParent);
    if (drag.mode === 'move') {
      const centerX = freeMoveCandidate.x + freeMoveCandidate.w * 0.5;
      const centerY = freeMoveCandidate.y + freeMoveCandidate.h * 0.5;
      if (!layoutContainsPoint(parentLayout, centerX, centerY)) return freeMoveCandidate;
      const snappedPos = resolveLayoutDropPosition(
        ctx,
        containerParent,
        { x: freeMoveCandidate.x, y: freeMoveCandidate.y },
        { w: drag.startLayout.w, h: drag.startLayout.h },
        moving.id
      );
      const maxY = Math.max(parentLayout.y, parentLayout.y + parentLayout.h - drag.startLayout.h);
      return clampLayout(
        {
          ...drag.startLayout,
          x: parentLayout.x,
          y: Math.max(parentLayout.y, Math.min(maxY, snappedPos.y)),
          w: parentLayout.w
        },
        cols
      );
    }
    const maxHeight = Math.max(1, parentLayout.y + parentLayout.h - drag.startLayout.y);
    const snappedHeight = Math.max(1, Math.round(freeResizeCandidate.h));
    return clampLayout(
      {
        ...drag.startLayout,
        x: parentLayout.x,
        y: drag.startLayout.y,
        w: parentLayout.w,
        h: Math.max(1, Math.min(maxHeight, snappedHeight))
      },
      cols
    );
  }

  if (drag.mode === 'move') {
    let next = snapMoveWithinContainer(ctx, moving, freeMoveCandidate, drag.startLayout);
    if (!containerParent) next = snapLayoutToCanvasCells(next, cols);
    return next;
  }
  let next = freeResizeCandidate;
  if (!containerParent) next = snapLayoutToCanvasCells(next, cols);
  return next;
}

/**
 * Applies one pointer sample to an in-progress drag. Returns the same object when nothing changed, a new
 * state when the layout or pointer moved, or null when the dragged widget no longer exists.
 */
export function advanceDrag(
  ctx: CanvasContext,
  drag: DragState,
  pointerX: number,
  pointerY: number,
  pointerOnCanvas: { x: number; y: number } | null,
  cellSize: number
): DragState | null {
  const moving = ctx.byId.get(drag.widgetId);
  if (!moving) return null;

  const next = dragCandidate(ctx, drag, moving, pointerX, pointerY, pointerOnCanvas, cellSize);
  const clamped = clampLayout(next, ctx.gridColumns);
  const pointerChanged = pointerX !== drag.pointerX || pointerY !== drag.pointerY;

  if (hasCollision(ctx, drag.widgetId, clamped)) {
    return pointerChanged ? { ...drag, pointerX, pointerY } : drag;
  }
  if (layoutsEqual(clamped, drag.currentLayout) && !pointerChanged) {
    return drag;
  }
  return { ...drag, pointerX, pointerY, currentLayout: clamped };
}
