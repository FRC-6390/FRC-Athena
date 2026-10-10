import { clampLayout, type DashboardWidget, type WidgetLayout } from '../dashboard';
import { readLayoutGridConfig } from '../widget-config';
import type { CanvasContext } from './model';

/** Only what container math reads; lets the context builder run before `layoutOf` exists. */
export type LayoutSource = Pick<CanvasContext, 'widgets' | 'byId' | 'gridColumns' | 'dragLayoutOf' | 'boundsOf'>;
import { absoluteFromGridCell, gridCellFromAbsolute, gridSpecFor, type GridLayoutSpec } from './grid-spec';
import { isContainerLayoutKind } from './tree';

/** Grid spec of a `layout_grid` container, from its current content bounds. Null for other kinds. */
export function gridSpecOf(ctx: LayoutSource, widget: DashboardWidget): GridLayoutSpec | null {
  if (widget.kind !== 'layout_grid') return null;
  return gridSpecFor(widget.config, ctx.boundsOf(widget));
}

/** Column/row counts a grid container shows, for the CSS background pattern. */
export function gridCountsOf(widget: DashboardWidget, layout: WidgetLayout): { columns: number; rows: number } {
  const cfg = readLayoutGridConfig(widget.config, Math.max(1, Math.floor(layout.w)), Math.max(1, Math.floor(layout.h)));
  return { columns: cfg.columns, rows: cfg.rows };
}

/** Children of each container, sorted by where they are being drawn (live drag included). */
export function childrenByContainer(ctx: LayoutSource): Map<string, DashboardWidget[]> {
  const byParent = new Map<string, DashboardWidget[]>();
  for (const widget of ctx.widgets) {
    const parentId = widget.parentLayoutId;
    if (!parentId) continue;
    const parent = ctx.byId.get(parentId);
    if (!parent || !isContainerLayoutKind(parent.kind)) continue;
    const siblings = byParent.get(parentId);
    if (siblings) {
      siblings.push(widget);
    } else {
      byParent.set(parentId, [widget]);
    }
  }
  for (const siblings of byParent.values()) {
    siblings.sort((a, b) => {
      const aLayout = ctx.dragLayoutOf(a);
      const bLayout = ctx.dragLayoutOf(b);
      return aLayout.y - bLayout.y || aLayout.x - bLayout.x || a.id.localeCompare(b.id);
    });
  }
  return byParent;
}

/**
 * Child layouts forced into their container: accordion children fill it, list children stack at full
 * width, grid children snap to cells. Keyed by child id.
 */
export function normalizedContainerLayouts(ctx: LayoutSource): Map<string, WidgetLayout> {
  const normalized = new Map<string, WidgetLayout>();
  const childrenByParent = childrenByContainer(ctx);

  for (const parent of ctx.widgets) {
    if (!isContainerLayoutKind(parent.kind)) continue;
    const children = childrenByParent.get(parent.id) ?? [];
    if (children.length === 0) continue;
    const parentLayout = ctx.boundsOf(parent);

    if (parent.kind === 'layout_accordion') {
      for (const child of children) {
        const base = ctx.dragLayoutOf(child);
        normalized.set(
          child.id,
          clampLayout({ ...base, x: parentLayout.x, y: parentLayout.y, w: parentLayout.w, h: parentLayout.h }, ctx.gridColumns)
        );
      }
      continue;
    }

    if (parent.kind === 'layout_list') {
      let cursorY = parentLayout.y;
      for (const child of children) {
        const base = ctx.dragLayoutOf(child);
        const childLayout = clampLayout(
          { ...base, x: parentLayout.x, y: cursorY, w: parentLayout.w },
          ctx.gridColumns
        );
        normalized.set(child.id, childLayout);
        cursorY = childLayout.y + childLayout.h;
      }
      continue;
    }

    if (parent.kind === 'layout_grid') {
      const spec = gridSpecOf(ctx, parent);
      for (const child of children) {
        const base = ctx.dragLayoutOf(child);
        if (!spec) {
          normalized.set(child.id, base);
          continue;
        }
        const cellLayout = gridCellFromAbsolute(spec, base);
        normalized.set(child.id, absoluteFromGridCell(spec, cellLayout, ctx.gridColumns) ?? base);
      }
    }
  }

  return normalized;
}

/** Accordion child fitted to the accordion's content bounds. */
export function normalizedAccordionChildLayout(
  ctx: LayoutSource,
  widget: DashboardWidget,
  fallback: WidgetLayout
): WidgetLayout | null {
  if (!widget.parentLayoutId) return null;
  const parent = ctx.byId.get(widget.parentLayoutId);
  if (!parent || parent.kind !== 'layout_accordion') return null;
  const parentLayout = ctx.boundsOf(parent);
  return clampLayout(
    { ...fallback, x: parentLayout.x, y: parentLayout.y, w: parentLayout.w, h: parentLayout.h },
    ctx.gridColumns
  );
}

/** List child placed at its stacking position among its siblings. */
export function normalizedListChildLayout(
  ctx: LayoutSource,
  widget: DashboardWidget,
  fallback: WidgetLayout
): WidgetLayout | null {
  if (!widget.parentLayoutId) return null;
  const parent = ctx.byId.get(widget.parentLayoutId);
  if (!parent || parent.kind !== 'layout_list') return null;

  const parentLayout = ctx.boundsOf(parent);
  const siblings = ctx.widgets
    .filter((entry) => entry.parentLayoutId === parent.id)
    .map((entry) => ({ id: entry.id, layout: ctx.dragLayoutOf(entry) }))
    .sort((a, b) => a.layout.y - b.layout.y || a.layout.x - b.layout.x || a.id.localeCompare(b.id));

  let cursorY = parentLayout.y;
  for (const sibling of siblings) {
    const normalized = clampLayout(
      { ...sibling.layout, x: parentLayout.x, y: cursorY, w: parentLayout.w },
      ctx.gridColumns
    );
    if (sibling.id === widget.id) return normalized;
    cursorY = normalized.y + normalized.h;
  }
  return fallback;
}

/** Grid child snapped to its cell. */
export function normalizedGridChildLayout(
  ctx: LayoutSource,
  widget: DashboardWidget,
  fallback: WidgetLayout
): WidgetLayout | null {
  if (!widget.parentLayoutId) return null;
  const parent = ctx.byId.get(widget.parentLayoutId);
  if (!parent || parent.kind !== 'layout_grid') return null;
  const spec = gridSpecOf(ctx, parent);
  if (!spec) return fallback;
  const cellLayout = gridCellFromAbsolute(spec, fallback);
  return absoluteFromGridCell(spec, cellLayout, ctx.gridColumns) ?? fallback;
}

/** Live measurement fallback: apply the axis normalisers a container child is subject to, in order. */
export function normalizeContainerAxisForBounds(
  ctx: LayoutSource,
  widget: DashboardWidget,
  fallback: WidgetLayout
): WidgetLayout {
  let normalized = fallback;
  normalized = normalizedAccordionChildLayout(ctx, widget, normalized) ?? normalized;
  normalized = normalizedListChildLayout(ctx, widget, normalized) ?? normalized;
  normalized = normalizedGridChildLayout(ctx, widget, normalized) ?? normalized;
  return normalized;
}
