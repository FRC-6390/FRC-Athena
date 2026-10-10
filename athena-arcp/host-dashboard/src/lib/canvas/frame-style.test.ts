import { describe, expect, test } from 'bun:test';
import { beginDragState } from './drag';
import { frameStyleFor } from './frame-style';
import { GRID_GAP, buildCanvasContext } from './model';
import { at, widget } from './test-support';

describe('card style', () => {
  const cell = 30;
  const step = cell + GRID_GAP;

  test('a card at rest is placed on its grid box with a trailing gap', () => {
    const widgets = [widget('a', 'metric', at(2, 1, 2, 1))];
    const ctx = buildCanvasContext({ widgets, gridColumns: 24, drag: null });
    expect(frameStyleFor({ ctx, drag: null, cellSize: cell, widget: widgets[0], layout: at(2, 1, 2, 1) })).toBe(
      `transform:translate3d(${step}px,0px,0);width:${2 * step - GRID_GAP}px;height:${cell}px;`
    );
  });

  test('the dragged card follows the pointer offset and is lifted', () => {
    const widgets = [widget('a', 'metric', at(2, 1, 2, 1))];
    const ctx = buildCanvasContext({ widgets, gridColumns: 24, drag: null });
    const drag = {
      ...beginDragState({
        target: widgets[0],
        mode: 'move',
        clientX: 100,
        clientY: 100,
        pointerOnCanvas: null,
        startLayout: at(2, 1, 2, 1),
        frozenLayoutBounds: new Map()
      }),
      pointerX: 130,
      pointerY: 90
    };
    const style = frameStyleFor({ ctx, drag, cellSize: cell, widget: widgets[0], layout: at(2, 1, 2, 1) });
    expect(style).toContain(`translate3d(${step + 30}px,-10px,0)`);
    expect(style).toContain('z-index:20;');
  });
});
