import { describe, expect, test } from 'bun:test';
import {
  absoluteFromGridCell,
  axisCellAtValue,
  axisCellFromRange,
  axisRangeFromCell,
  buildGridAxisEdges,
  gridCellFromAbsolute,
  gridSpecFor
} from './grid-spec';
import { at } from './test-support';

describe('grid axis math', () => {
  test('edges split the parent span evenly', () => {
    expect(buildGridAxisEdges(1, 6, 3)).toEqual([1, 3, 5, 7]);
  });

  test('a value resolves to its cell and clamps at both ends', () => {
    const edges = buildGridAxisEdges(1, 6, 3);
    expect(axisCellAtValue(0, edges)).toBe(1);
    expect(axisCellAtValue(2.9, edges)).toBe(1);
    expect(axisCellAtValue(3, edges)).toBe(2);
    expect(axisCellAtValue(99, edges)).toBe(3);
  });

  test('a range covers the cells it mostly overlaps and never less than one', () => {
    const edges = buildGridAxisEdges(1, 6, 3);
    expect(axisCellFromRange(1.2, 3.1, edges)).toEqual({ index: 1, span: 2 });
    expect(axisCellFromRange(1.2, 5.8, edges)).toEqual({ index: 1, span: 3 });
    expect(axisCellFromRange(3.5, 3.5, edges)).toEqual({ index: 2, span: 1 });
  });

  test('cell ranges round-trip to the same absolute box for whole cells', () => {
    const range = axisRangeFromCell(2, 2, buildGridAxisEdges(1, 6, 3));
    expect(range.start).toBeCloseTo(3);
    expect(range.span).toBeCloseTo(4);
  });
});

describe('grid container specs', () => {
  test('autoSize follows the container bounds; explicit counts win otherwise', () => {
    const bounds = at(1, 1, 8, 4);
    const auto = gridSpecFor(undefined, bounds);
    expect(auto.columns).toBe(8);
    expect(auto.rows).toBe(4);

    const fixed = gridSpecFor({ columns: 3, rows: 2, autoSize: false }, bounds);
    expect(fixed.columns).toBe(3);
    expect(fixed.rows).toBe(2);
    expect(fixed.xEdges).toHaveLength(4);
  });

  test('absolute and cell layouts convert both ways', () => {
    const spec = gridSpecFor({ columns: 4, rows: 2, autoSize: false }, at(1, 1, 8, 4));
    const cell = gridCellFromAbsolute(spec, at(3, 2, 2, 2));
    expect(cell).toEqual({ x: 2, y: 1, w: 1, h: 2 });
    const back = absoluteFromGridCell(spec, cell, 24);
    expect(back.x).toBeCloseTo(3);
    expect(back.y).toBeCloseTo(1);
    expect(back.w).toBeCloseTo(2);
    expect(back.h).toBeCloseTo(4);
  });
});
