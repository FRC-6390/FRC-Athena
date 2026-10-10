import { clampLayout, isLayoutWidgetKind, type DashboardWidget, type WidgetLayout } from '../dashboard';
import type { CanvasContext, DragMode } from './model';
import { absoluteFromGridCell, gridCellFromAbsolute } from './grid-spec';
import { gridSpecOf } from './containers';
import { layoutContainsPoint } from './grid-math';
import { resolveLayoutDropPosition } from './placement';
import { findContainerParent } from './tree';

/** Rounds a free-moved or free-resized layout onto whole canvas cells (used when not inside a container). */
export function snapLayoutToCanvasCells(layout: WidgetLayout, gridColumns: number): WidgetLayout {
  return clampLayout(
    {
      ...layout,
      x: Math.round(layout.x),
      y: Math.round(layout.y),
      w: Math.max(1, Math.round(layout.w)),
      h: Math.max(1, Math.round(layout.h))
    },
    gridColumns
  );
}

/**
 * While a widget is dragged over a container, snap its candidate to that container's slots. A widget whose
 * centre leaves the container is released and keeps the free candidate.
 */
export function snapMoveWithinContainer(
  ctx: CanvasContext,
  moving: DashboardWidget,
  candidate: WidgetLayout,
  startLayout: WidgetLayout
): WidgetLayout {
  if (isLayoutWidgetKind(moving.kind)) return candidate;
  const parent = findContainerParent(ctx.byId, moving);
  if (!parent) return candidate;

  const parentLayout =
    parent.kind === 'layout_grid' ? (gridSpecOf(ctx, parent)?.parent ?? ctx.boundsOf(parent)) : ctx.boundsOf(parent);
  const candidateCenterX = candidate.x + candidate.w * 0.5;
  const candidateCenterY = candidate.y + candidate.h * 0.5;

  // Let widgets leave a container when their center is dragged outside.
  if (!layoutContainsPoint(parentLayout, candidateCenterX, candidateCenterY)) return candidate;

  if (parent.kind === 'layout_accordion') {
    return clampLayout(
      { ...candidate, x: parentLayout.x, y: parentLayout.y, w: parentLayout.w, h: parentLayout.h },
      ctx.gridColumns
    );
  }

  if (parent.kind === 'layout_list') {
    const snappedPos = resolveLayoutDropPosition(
      ctx,
      parent,
      { x: candidate.x, y: candidate.y },
      { w: candidate.w, h: candidate.h },
      moving.id
    );
    return clampLayout({ ...candidate, x: parentLayout.x, y: snappedPos.y, w: parentLayout.w }, ctx.gridColumns);
  }

  const spec = gridSpecOf(ctx, parent);
  if (!spec) return candidate;
  const startCell = gridCellFromAbsolute(spec, startLayout);
  const candidateCell = gridCellFromAbsolute(spec, candidate);

  const snappedCell: WidgetLayout = {
    x: Math.max(1, Math.min(Math.max(1, spec.columns - startCell.w + 1), candidateCell.x)),
    y: Math.max(1, Math.min(Math.max(1, spec.rows - startCell.h + 1), candidateCell.y)),
    w: startCell.w,
    h: startCell.h
  };
  return absoluteFromGridCell(spec, snappedCell, ctx.gridColumns) ?? candidate;
}

/** Keeps a grid child on its cell grid after a drop that did not change its parent. */
export function snapLayoutToExistingParentGrid(
  ctx: CanvasContext,
  moving: DashboardWidget,
  layout: WidgetLayout,
  startLayout: WidgetLayout,
  mode: DragMode
): WidgetLayout {
  const parent = findContainerParent(ctx.byId, moving);
  if (parent?.kind !== 'layout_grid') return layout;
  const spec = gridSpecOf(ctx, parent);
  if (!spec) return layout;
  const startCell = gridCellFromAbsolute(spec, startLayout);
  const candidateCell = gridCellFromAbsolute(spec, layout);

  if (mode === 'move') {
    const maxCol = Math.max(1, spec.columns - startCell.w + 1);
    const maxRow = Math.max(1, spec.rows - startCell.h + 1);
    const snapped = absoluteFromGridCell(
      spec,
      {
        x: Math.max(1, Math.min(maxCol, candidateCell.x)),
        y: Math.max(1, Math.min(maxRow, candidateCell.y)),
        w: startCell.w,
        h: startCell.h
      },
      ctx.gridColumns
    );
    return snapped ?? layout;
  }

  const maxW = Math.max(1, spec.columns - startCell.x + 1);
  const maxH = Math.max(1, spec.rows - startCell.y + 1);
  const snapped = absoluteFromGridCell(
    spec,
    {
      x: startCell.x,
      y: startCell.y,
      w: Math.max(1, Math.min(maxW, candidateCell.w)),
      h: Math.max(1, Math.min(maxH, candidateCell.h))
    },
    ctx.gridColumns
  );
  return snapped ?? layout;
}

/** Keeps a list or accordion child inside its container after a drop that did not change its parent. */
export function snapLayoutToExistingParentList(
  ctx: CanvasContext,
  moving: DashboardWidget,
  layout: WidgetLayout,
  startLayout: WidgetLayout,
  mode: DragMode
): WidgetLayout {
  const parent = findContainerParent(ctx.byId, moving);
  if (parent?.kind !== 'layout_list' && parent?.kind !== 'layout_accordion') return layout;
  const parentLayout = ctx.boundsOf(parent);

  if (parent.kind === 'layout_accordion') {
    if (mode === 'move') {
      const centerX = layout.x + layout.w * 0.5;
      const centerY = layout.y + layout.h * 0.5;
      if (!layoutContainsPoint(parentLayout, centerX, centerY)) return layout;
    }
    return clampLayout(
      { ...layout, x: parentLayout.x, y: parentLayout.y, w: parentLayout.w, h: parentLayout.h },
      ctx.gridColumns
    );
  }

  if (mode === 'move') {
    const centerX = layout.x + layout.w * 0.5;
    const centerY = layout.y + layout.h * 0.5;
    if (!layoutContainsPoint(parentLayout, centerX, centerY)) return layout;
    const snappedPos = resolveLayoutDropPosition(
      ctx,
      parent,
      { x: layout.x, y: layout.y },
      { w: layout.w, h: layout.h },
      moving.id
    );
    const maxY = Math.max(parentLayout.y, parentLayout.y + parentLayout.h - layout.h);
    return clampLayout(
      {
        ...layout,
        x: parentLayout.x,
        y: Math.max(parentLayout.y, Math.min(maxY, snappedPos.y)),
        w: parentLayout.w
      },
      ctx.gridColumns
    );
  }

  const maxHeight = Math.max(1, parentLayout.y + parentLayout.h - startLayout.y);
  const snappedHeight = Math.max(1, Math.round(layout.h));
  return clampLayout(
    {
      ...layout,
      x: parentLayout.x,
      y: startLayout.y,
      w: parentLayout.w,
      h: Math.max(1, Math.min(maxHeight, snappedHeight))
    },
    ctx.gridColumns
  );
}
