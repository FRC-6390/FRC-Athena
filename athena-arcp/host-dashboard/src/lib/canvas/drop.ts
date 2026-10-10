import { clampLayout, isLayoutWidgetKind, type DashboardWidget, type WidgetLayout } from '../dashboard';
import { DEFAULT_SIGNAL_DROP_SIZE, type CanvasContext } from './model';
import { absoluteFromGridCell, gridCellFromAbsolute } from './grid-spec';
import { gridSpecOf } from './containers';
import { layoutsEqual } from './grid-math';
import { findLayoutWidgetAtCell, resolveLayoutDropPosition } from './placement';
import { hasCollision, type DragState } from './drag';
import {
  snapLayoutToCanvasCells,
  snapLayoutToExistingParentGrid,
  snapLayoutToExistingParentList
} from './snap';
import {
  accordionHasOtherChildren,
  findContainerParent,
  isContainerLayoutKind,
  wouldCreateParentCycle
} from './tree';

export type DropPatch = {
  widgetId: string;
  layout: WidgetLayout;
  /** `undefined` keeps the parent; `null` moves the widget onto the canvas. */
  parentLayoutId: string | null | undefined;
};

/** Where a dropped widget is placed in grid units and which container (if any) receives it. */
export type SurfaceDropPlan = {
  position: Pick<WidgetLayout, 'x' | 'y'>;
  parentLayoutId: string | null;
};

/**
 * Container receiving a dragged widget for the given layout, when its centre is over one that accepts it.
 * Returns the adjusted layout inside that container, or null when the target does not accept the drop.
 */
function layoutInsideTarget(
  ctx: CanvasContext,
  moving: DashboardWidget,
  layoutTarget: DashboardWidget,
  finalLayout: WidgetLayout,
  startLayout: WidgetLayout
): WidgetLayout {
  const snappedPos = resolveLayoutDropPosition(
    ctx,
    layoutTarget,
    { x: finalLayout.x, y: finalLayout.y },
    { w: finalLayout.w, h: finalLayout.h },
    moving.id
  );

  if (layoutTarget.kind === 'layout_accordion') {
    const targetLayout = ctx.boundsOf(layoutTarget);
    return { ...finalLayout, x: targetLayout.x, y: targetLayout.y, w: targetLayout.w, h: targetLayout.h };
  }

  if (layoutTarget.kind === 'layout_list') {
    const targetLayout = ctx.boundsOf(layoutTarget);
    return { ...finalLayout, ...snappedPos, x: targetLayout.x, w: targetLayout.w };
  }

  // Grid: the widget keeps its size when it stays in the same grid, otherwise its drawn size.
  const sourceGridParent = findContainerParent(ctx.byId, moving);
  const movingWithinSameGrid = sourceGridParent?.kind === 'layout_grid' && sourceGridParent.id === layoutTarget.id;
  const spec = gridSpecOf(ctx, layoutTarget);
  if (!spec) return { ...finalLayout, ...snappedPos };
  const startCell = gridCellFromAbsolute(spec, startLayout);
  const dropAnchorCell = gridCellFromAbsolute(spec, { x: snappedPos.x, y: snappedPos.y, w: 1, h: 1 });
  const sourceSizeCell = gridCellFromAbsolute(spec, { ...finalLayout, x: finalLayout.x, y: finalLayout.y });
  const desiredW = movingWithinSameGrid ? startCell.w : (sourceSizeCell?.w ?? 1);
  const desiredH = movingWithinSameGrid ? startCell.h : (sourceSizeCell?.h ?? 1);
  const maxW = Math.max(1, spec.columns - dropAnchorCell.x + 1);
  const maxH = Math.max(1, spec.rows - dropAnchorCell.y + 1);
  const snappedCell: WidgetLayout = {
    x: dropAnchorCell.x,
    y: dropAnchorCell.y,
    w: Math.max(1, Math.min(maxW, desiredW)),
    h: Math.max(1, Math.min(maxH, desiredH))
  };
  return absoluteFromGridCell(spec, snappedCell, ctx.gridColumns);
}

/**
 * Outcome of releasing a drag: the final layout, the parent it lands in, and whether anything changed. Null
 * when the dragged widget no longer exists.
 */
export function planDrop(
  ctx: CanvasContext,
  drag: DragState
): (DropPatch & { patch: boolean }) | null {
  const { widgetId, startLayout, currentLayout, mode } = drag;
  const moving = ctx.byId.get(widgetId);
  if (!moving) return null;

  let finalLayout = currentLayout;
  let parentLayoutId: string | null | undefined = undefined;

  if (mode === 'move') {
    const layoutTarget = findLayoutWidgetAtCell(
      ctx,
      { x: finalLayout.x + finalLayout.w * 0.5, y: finalLayout.y + finalLayout.h * 0.5 },
      widgetId,
      isLayoutWidgetKind(moving.kind) ? moving.id : undefined
    );
    const targetAcceptsDrop =
      layoutTarget?.kind !== 'layout_accordion' ||
      !accordionHasOtherChildren(ctx.widgets, layoutTarget.id, widgetId) ||
      layoutTarget.id === moving.parentLayoutId;
    if (
      layoutTarget &&
      targetAcceptsDrop &&
      isContainerLayoutKind(layoutTarget.kind) &&
      !wouldCreateParentCycle(ctx.byId, widgetId, layoutTarget.id)
    ) {
      const adjusted = layoutInsideTarget(ctx, moving, layoutTarget, finalLayout, startLayout);
      const snapped = clampLayout(adjusted, ctx.gridColumns);
      if (!hasCollision(ctx, widgetId, snapped)) {
        finalLayout = snapped;
        parentLayoutId = layoutTarget.id;
      }
    } else if (!layoutTarget) {
      parentLayoutId = null;
    }
  }

  // Parent unchanged: keep the widget on its container's slots.
  if (parentLayoutId === undefined) {
    const sourceParent = findContainerParent(ctx.byId, moving);
    if (sourceParent?.kind === 'layout_list' || sourceParent?.kind === 'layout_accordion') {
      finalLayout = snapLayoutToExistingParentList(ctx, moving, finalLayout, startLayout, mode);
    } else if (sourceParent?.kind === 'layout_grid') {
      finalLayout = snapLayoutToExistingParentGrid(ctx, moving, finalLayout, startLayout, mode);
    }
  }

  const effectiveParentId = parentLayoutId === undefined ? (moving.parentLayoutId ?? null) : parentLayoutId;
  const effectiveParent = effectiveParentId ? (ctx.byId.get(effectiveParentId) ?? null) : null;
  if (!effectiveParent) {
    finalLayout = snapLayoutToCanvasCells(finalLayout, ctx.gridColumns);
  }

  const parentChanged =
    parentLayoutId !== undefined && (moving.parentLayoutId ?? null) !== (parentLayoutId ?? null);
  return {
    widgetId,
    layout: finalLayout,
    parentLayoutId,
    patch: !layoutsEqual(startLayout, finalLayout) || parentChanged
  };
}

/** Drop from outside the canvas (signal, topic or layout tool) at a grid position. */
export function planSurfaceDrop(ctx: CanvasContext, point: { x: number; y: number }): SurfaceDropPlan {
  const fallbackPosition = { x: Math.max(1, point.x), y: Math.max(1, point.y) };
  const canvasSnappedPosition = {
    x: Math.max(1, Math.round(point.x)),
    y: Math.max(1, Math.round(point.y))
  };
  const rawLayoutDropTarget = findLayoutWidgetAtCell(ctx, fallbackPosition);
  const layoutDropTarget =
    rawLayoutDropTarget?.kind === 'layout_accordion' && accordionHasOtherChildren(ctx.widgets, rawLayoutDropTarget.id)
      ? null
      : rawLayoutDropTarget;
  const position = layoutDropTarget
    ? resolveLayoutDropPosition(ctx, layoutDropTarget, fallbackPosition, DEFAULT_SIGNAL_DROP_SIZE)
    : canvasSnappedPosition;
  const parentLayoutId =
    layoutDropTarget && isContainerLayoutKind(layoutDropTarget.kind) ? layoutDropTarget.id : null;
  return { position, parentLayoutId };
}
