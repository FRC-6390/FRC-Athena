import { isLayoutWidgetKind, type DashboardWidget, type WidgetConfigRecord } from '../dashboard';
import { readLayoutAccordionConfig } from '../widget-config';

export function isContainerLayoutKind(kind: DashboardWidget['kind']): boolean {
  return kind === 'layout_list' || kind === 'layout_grid' || kind === 'layout_accordion';
}

export function isAccordionCollapsed(widget: DashboardWidget | null | undefined): boolean {
  if (!widget || widget.kind !== 'layout_accordion') return false;
  return readLayoutAccordionConfig(widget.config).collapsed;
}

/** Config to store when an accordion header is clicked: the same config with `collapsed` flipped. */
export function toggledAccordionConfig(widget: DashboardWidget): WidgetConfigRecord {
  const baseConfig =
    widget.config && typeof widget.config === 'object' && !Array.isArray(widget.config) ? widget.config : {};
  const nextCollapsed = !readLayoutAccordionConfig(widget.config).collapsed;
  return { ...baseConfig, collapsed: nextCollapsed };
}

/**
 * Draw order: containers first, each followed by its children (depth-first), siblings sorted by position.
 * Widgets whose parent is missing are treated as roots; anything unreachable is appended last.
 */
export function renderOrder(
  widgets: readonly DashboardWidget[],
  byId: ReadonlyMap<string, DashboardWidget>
): DashboardWidget[] {
  const siblingCompare = (a: DashboardWidget, b: DashboardWidget): number => {
    const aLayout = isLayoutWidgetKind(a.kind);
    const bLayout = isLayoutWidgetKind(b.kind);
    if (aLayout !== bLayout) return aLayout ? -1 : 1;
    return a.layout.y - b.layout.y || a.layout.x - b.layout.x || a.id.localeCompare(b.id);
  };

  const childrenByParent = new Map<string | null, DashboardWidget[]>();
  for (const widget of widgets) {
    const parentKey = widget.parentLayoutId && byId.has(widget.parentLayoutId) ? widget.parentLayoutId : null;
    const children = childrenByParent.get(parentKey);
    if (children) {
      children.push(widget);
    } else {
      childrenByParent.set(parentKey, [widget]);
    }
  }
  for (const children of childrenByParent.values()) {
    children.sort(siblingCompare);
  }

  const ordered: DashboardWidget[] = [];
  const visited = new Set<string>();
  const visit = (widget: DashboardWidget) => {
    if (visited.has(widget.id)) return;
    visited.add(widget.id);
    ordered.push(widget);
    for (const child of childrenByParent.get(widget.id) ?? []) {
      visit(child);
    }
  };

  for (const root of childrenByParent.get(null) ?? []) {
    visit(root);
  }
  if (ordered.length < widgets.length) {
    for (const widget of [...widgets].sort(siblingCompare)) {
      visit(widget);
    }
  }
  return ordered;
}

/** Widget ids that sit under a collapsed accordion (directly or through nested containers). */
export function hiddenByCollapsedAccordion(
  widgets: readonly DashboardWidget[],
  byId: ReadonlyMap<string, DashboardWidget>
): Map<string, boolean> {
  const hidden = new Map<string, boolean>();
  for (const widget of widgets) {
    let isHidden = false;
    let cursor = widget.parentLayoutId;
    const seen = new Set<string>();
    while (cursor) {
      if (seen.has(cursor)) break;
      seen.add(cursor);
      const ancestor = byId.get(cursor);
      if (ancestor && isAccordionCollapsed(ancestor)) {
        isHidden = true;
        break;
      }
      cursor = byId.get(cursor)?.parentLayoutId;
    }
    hidden.set(widget.id, isHidden);
  }
  return hidden;
}

export function isDescendantOf(
  byId: ReadonlyMap<string, DashboardWidget>,
  widget: DashboardWidget,
  ancestorId: string
): boolean {
  let cursor = widget.parentLayoutId;
  while (cursor) {
    if (cursor === ancestorId) return true;
    cursor = byId.get(cursor)?.parentLayoutId;
  }
  return false;
}

/** The container a widget currently belongs to, or null when it sits on the canvas. */
export function findContainerParent(
  byId: ReadonlyMap<string, DashboardWidget>,
  widget: DashboardWidget
): DashboardWidget | null {
  const parentId = widget.parentLayoutId;
  if (!parentId) return null;
  const parent = byId.get(parentId);
  if (!parent || !isContainerLayoutKind(parent.kind)) return null;
  return parent;
}

export function wouldCreateParentCycle(
  byId: ReadonlyMap<string, DashboardWidget>,
  widgetId: string,
  candidateParentId: string
): boolean {
  let cursor: string | undefined = candidateParentId;
  while (cursor) {
    if (cursor === widgetId) return true;
    cursor = byId.get(cursor)?.parentLayoutId;
  }
  return false;
}

/** Number of expanded accordions in the chain from this widget to the root (drives stacking order). */
export function accordionStackLayer(
  byId: ReadonlyMap<string, DashboardWidget>,
  widget: DashboardWidget
): number {
  let layer = 0;
  let first = true;
  let cursor: DashboardWidget | null = widget;
  while (cursor) {
    if (cursor.kind === 'layout_accordion' && !isAccordionCollapsed(cursor)) {
      // Accordion shell floats above baseline; descendants render above shell.
      layer += first ? 2 : 3;
    }
    first = false;
    const parentId: string | undefined = cursor.parentLayoutId;
    cursor = parentId ? byId.get(parentId) ?? null : null;
  }
  return layer;
}

export function accordionHasOtherChildren(
  widgets: readonly DashboardWidget[],
  accordionId: string,
  ignoreWidgetId?: string
): boolean {
  return widgets.some(
    (widget) =>
      widget.parentLayoutId === accordionId &&
      (ignoreWidgetId === undefined || widget.id !== ignoreWidgetId)
  );
}
