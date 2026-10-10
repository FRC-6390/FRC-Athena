import { describe, expect, test } from 'bun:test';
import { advanceDrag, beginDragState, dragTargetFor, hasCollision, type DragState } from './drag';
import { planDrop, planSurfaceDrop } from './drop';
import { buildCanvasContext } from './model';
import { gridColumnsFor } from './grid-math';
import { snapLayoutToCanvasCells } from './snap';
import { at, widget } from './test-support';
import type { DashboardWidget } from '../dashboard';

// Canvas interaction tests use the same context builder as the component, with DOM bounds replaced by
// stored container layouts. Coordinates are grid units.

function contextFor(widgets: DashboardWidget[], drag: DragState | null = null) {
  return buildCanvasContext({ widgets, gridColumns: gridColumnsFor(widgets), drag });
}

function startDrag(
  widgets: DashboardWidget[],
  id: string,
  mode: 'move' | 'resize',
  grab: { x: number; y: number }
): DragState {
  const ctx = contextFor(widgets);
  const target = dragTargetFor(ctx, ctx.byId.get(id)!, mode);
  return beginDragState({
    target,
    mode,
    clientX: 0,
    clientY: 0,
    pointerOnCanvas: grab,
    startLayout: ctx.layoutOf(target),
    frozenLayoutBounds: new Map()
  });
}

/** Moves the pointer to a grid position in one sample. */
function moveTo(widgets: DashboardWidget[], drag: DragState, point: { x: number; y: number }): DragState {
  const ctx = contextFor(widgets, drag);
  const next = advanceDrag(ctx, drag, point.x * 10, point.y * 10, point, 10);
  return next ?? drag;
}

function drop(widgets: DashboardWidget[], drag: DragState) {
  const ctx = contextFor(widgets, drag);
  return planDrop(ctx, drag);
}

describe('free move on the canvas', () => {
  test('moves snap to whole cells and stay off other widgets', () => {
    const widgets = [widget('a', 'metric', at(1, 1, 2, 1)), widget('b', 'metric', at(6, 1, 2, 1))];
    let drag = startDrag(widgets, 'a', 'move', { x: 1, y: 1 });
    drag = moveTo(widgets, drag, { x: 3.4, y: 2.4 });
    const patch = drop(widgets, drag);
    expect(patch?.patch).toBe(true);
    expect(patch?.parentLayoutId).toBeNull();
    expect(patch?.layout).toEqual(at(3, 2, 2, 1));
  });

  test('a move onto another data widget is rejected and the last free layout is kept', () => {
    const widgets = [widget('a', 'metric', at(1, 1, 2, 1)), widget('b', 'metric', at(6, 1, 2, 1))];
    const ctx = contextFor(widgets);
    expect(hasCollision(ctx, 'a', at(5, 1, 2, 1))).toBe(true);
    let drag = startDrag(widgets, 'a', 'move', { x: 1, y: 1 });
    drag = moveTo(widgets, drag, { x: 5.2, y: 1.1 });
    expect(drag.currentLayout).toEqual(at(1, 1, 2, 1));
  });

  test('releasing where nothing changed produces no patch', () => {
    const widgets = [widget('a', 'metric', at(1, 1, 2, 1))];
    const drag = startDrag(widgets, 'a', 'move', { x: 1, y: 1 });
    expect(drop(widgets, drag)?.patch).toBe(false);
  });

  test('a dropped widget that vanished produces no plan', () => {
    const widgets = [widget('a', 'metric', at(1, 1, 2, 1))];
    const drag = startDrag(widgets, 'a', 'move', { x: 1, y: 1 });
    expect(drop([], drag)).toBeNull();
  });
});

describe('resize on the canvas', () => {
  test('resizing grows width and height in whole cells', () => {
    const widgets = [widget('a', 'metric', at(1, 1, 2, 1))];
    let drag = startDrag(widgets, 'a', 'resize', { x: 3, y: 2 });
    drag = moveTo(widgets, drag, { x: 5.1, y: 3.2 });
    expect(drag.currentLayout.w).toBe(4);
    expect(drag.currentLayout.h).toBe(2);
  });
});

describe('lists and accordions', () => {
  const list = widget('list', 'layout_list', at(1, 1, 6, 6));

  test('dropping a widget into a list stacks it at full list width', () => {
    const widgets = [list, widget('a', 'metric', at(9, 1, 2, 1))];
    let drag = startDrag(widgets, 'a', 'move', { x: 9, y: 1 });
    drag = moveTo(widgets, drag, { x: 2, y: 2 });
    const patch = drop(widgets, drag);
    expect(patch?.parentLayoutId).toBe('list');
    expect(patch?.layout.x).toBe(1);
    expect(patch?.layout.w).toBe(6);
  });

  test('a child stays in its list when dropped inside it again', () => {
    const widgets = [list, widget('a', 'metric', at(1, 1, 6, 1), { parentLayoutId: 'list' })];
    let drag = startDrag(widgets, 'a', 'move', { x: 2, y: 1 });
    drag = moveTo(widgets, drag, { x: 2, y: 3 });
    const patch = drop(widgets, drag);
    expect(patch?.parentLayoutId).toBe('list');
  });

  test('accordions take a single child: a drop onto an occupied accordion keeps the widget off it', () => {
    const acc = widget('acc', 'layout_accordion', at(1, 1, 6, 3), { config: { collapsed: false } });
    const occupant = widget('x', 'metric', at(1, 1, 2, 1), { parentLayoutId: 'acc' });
    const mover = widget('a', 'metric', at(9, 1, 2, 1));
    const widgets = [acc, occupant, mover];
    const drag = startDrag(widgets, 'a', 'move', { x: 9, y: 1 });
    // Pointer over the occupied accordion: the drop is not accepted, so the parent is left unchanged.
    const attempt = { ...drag, currentLayout: at(2, 2, 2, 1) };
    expect(drop(widgets, attempt)?.parentLayoutId).toBeUndefined();
    // Collision with the accordion's child also blocks the live move itself.
    expect(moveTo(widgets, drag, { x: 2, y: 2 }).currentLayout).toEqual(at(9, 1, 2, 1));
  });

  test('dropping a layout block into its own descendant is refused', () => {
    const outer = widget('outer', 'layout_list', at(1, 1, 6, 6));
    const inner = widget('inner', 'layout_list', at(1, 1, 6, 3), { parentLayoutId: 'outer' });
    const widgets = [outer, inner];
    let drag = startDrag(widgets, 'outer', 'move', { x: 2, y: 2 });
    drag = moveTo(widgets, drag, { x: 2, y: 2 });
    const patch = drop(widgets, drag);
    expect(patch?.parentLayoutId).not.toBe('inner');
  });
});

describe('grid containers', () => {
  const grid = widget('grid', 'layout_grid', at(1, 1, 8, 4), { config: { columns: 4, rows: 2, autoSize: false } });

  test('dropping into a grid snaps the widget to the cell under the pointer', () => {
    const widgets = [grid, widget('a', 'metric', at(12, 1, 2, 1))];
    let drag = startDrag(widgets, 'a', 'move', { x: 12, y: 1 });
    drag = moveTo(widgets, drag, { x: 6, y: 4 });
    const patch = drop(widgets, drag);
    expect(patch?.parentLayoutId).toBe('grid');
    // Cell columns are 2 grid units wide; cell (col 3, row 2) starts at x=5, y=3.
    expect(patch?.layout.x).toBeCloseTo(5);
    expect(patch?.layout.y).toBeCloseTo(3);
  });

  test('a grid child moved within the grid keeps its cell size', () => {
    const widgets = [grid, widget('a', 'metric', at(1, 1, 2, 2), { parentLayoutId: 'grid' })];
    let drag = startDrag(widgets, 'a', 'move', { x: 1, y: 1 });
    drag = moveTo(widgets, drag, { x: 6, y: 3 });
    const patch = drop(widgets, drag);
    expect(patch?.parentLayoutId).toBe('grid');
    // A 2x2 child fills exactly one cell (cells are 2x2 grid units here), so it moves to cell (3, 2).
    expect(patch?.layout).toEqual(at(5, 3, 2, 2));
  });
});

describe('drops from outside the canvas', () => {
  test('outside any container the drop snaps to the rounded canvas cell', () => {
    const widgets = [widget('a', 'metric', at(1, 1, 2, 1))];
    const plan = planSurfaceDrop(contextFor(widgets), { x: 4.4, y: 2.6 });
    expect(plan).toEqual({ position: { x: 4, y: 3 }, parentLayoutId: null });
  });

  test('a drop over a list lands in the list at its left edge', () => {
    const widgets = [widget('list', 'layout_list', at(3, 2, 6, 4))];
    const plan = planSurfaceDrop(contextFor(widgets), { x: 4.2, y: 3.1 });
    expect(plan.parentLayoutId).toBe('list');
    expect(plan.position.x).toBe(3);
  });

  test('a drop onto a text block lands below its header and parents nothing', () => {
    const widgets = [widget('title', 'layout_title', at(1, 1, 6, 2))];
    const plan = planSurfaceDrop(contextFor(widgets), { x: 2, y: 1.5 });
    expect(plan).toEqual({ position: { x: 1, y: 2 }, parentLayoutId: null });
  });
});

describe('snap helper', () => {
  test('canvas snap rounds and keeps a minimum size of one cell', () => {
    expect(snapLayoutToCanvasCells(at(1.4, 2.6, 0.2, 0.4), 24)).toEqual(at(1, 3, 1, 1));
  });
});

