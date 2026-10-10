import { describe, expect, test } from 'bun:test';
import { DEFAULT_GRID_COLUMNS } from '../dashboard';
import {
  canvasRowsFor,
  cellSizeFor,
  contentSizeFor,
  gridColumnsFor,
  pixelRectFor,
  pointToGridUnits,
  renderedExtent,
  scrollMetricsFor,
  visibleRowsFor
} from './grid-math';
import { GRID_GAP, MIN_CELL_SIZE } from './model';
import { at, widget } from './test-support';

describe('grid columns and sizes', () => {
  test('columns never drop below the default and grow with the rightmost widget', () => {
    expect(gridColumnsFor([])).toBe(DEFAULT_GRID_COLUMNS);
    expect(gridColumnsFor([widget('a', 'metric', at(20, 1, 10, 1))])).toBe(30);
    expect(gridColumnsFor([widget('a', 'metric', at(90, 1, 40, 1))])).toBe(96);
  });

  test('cell size shrinks with more columns but never below the minimum', () => {
    expect(cellSizeFor(1000, 24)).toBeCloseTo((1000 - 23 * GRID_GAP) / 24);
    expect(cellSizeFor(10, 96)).toBe(MIN_CELL_SIZE);
  });

  test('visible rows fill the viewport height and canvas rows cover every widget', () => {
    const cell = 40;
    expect(visibleRowsFor(400, cell)).toBe(Math.floor((400 + GRID_GAP) / (cell + GRID_GAP)));
    expect(canvasRowsFor([widget('a', 'metric', at(1, 30, 2, 5))], 4)).toBe(35);
    expect(canvasRowsFor([], 2)).toBe(8);
    expect(contentSizeFor(24, 10, 40).width).toBe(24 * 40 + 23 * GRID_GAP);
  });
});

describe('pixel boxes', () => {
  test('top-level widgets keep the trailing gap, container children fill their cell', () => {
    const cell = 30;
    const step = cell + GRID_GAP;
    expect(pixelRectFor(at(2, 3, 2, 1), cell, false)).toEqual({
      left: step,
      top: 2 * step,
      width: 2 * step - GRID_GAP,
      height: step - GRID_GAP
    });
    expect(pixelRectFor(at(2, 3, 2, 1), cell, true)).toEqual({
      left: step,
      top: 2 * step,
      width: 2 * step,
      height: step
    });
  });

  test('pointer to grid units accounts for the surface offset and scroll', () => {
    const cell = 30;
    const step = cell + GRID_GAP;
    const units = pointToGridUnits(110, 60, { left: 10, top: 20, scrollLeft: 0, scrollTop: 0 }, cell);
    expect(units?.x).toBeCloseTo(100 / step + 1);
    expect(units?.y).toBeCloseTo(40 / step + 1);
    const scrolled = pointToGridUnits(110, 60, { left: 10, top: 20, scrollLeft: step, scrollTop: 0 }, cell);
    expect(scrolled?.x).toBeCloseTo(units!.x + 1);
  });
});

describe('scroll extent', () => {
  test('overflow on one axis reserves a scrollbar gutter on the other', () => {
    const content = { width: 500, height: 300 };
    const rendered = renderedExtent([{ layout: at(1, 1, 2, 2), withoutTrailingGap: false }], 30);
    const wide = scrollMetricsFor(content, { maxRightPx: 800, maxBottomPx: 100 }, 600, 400);
    expect(wide.overflowX).toBe(true);
    expect(wide.overflowY).toBe(false);
    expect(wide.scrollHeight).toBe(300 + 18);
    expect(wide.scrollWidth).toBe(800);

    const tall = scrollMetricsFor(content, rendered, 600, 200);
    expect(tall.overflowY).toBe(true);
    expect(tall.scrollWidth).toBe(500 + 18);
  });
});
