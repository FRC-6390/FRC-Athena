import { clampLayout, type WidgetConfigRecord, type WidgetLayout } from '../dashboard';
import { readLayoutGridConfig } from '../widget-config';

/** A `layout_grid` container: its bounds, its cell counts and the pixel-free edge positions of each cell. */
export type GridLayoutSpec = {
  parent: WidgetLayout;
  columns: number;
  rows: number;
  xEdges: number[];
  yEdges: number[];
};

export function buildGridAxisEdges(start: number, span: number, segments: number): number[] {
  const count = Math.max(1, Math.floor(segments));
  const edges: number[] = [];
  for (let index = 0; index <= count; index++) {
    edges.push(start + (index * span) / count);
  }
  return edges;
}

/** Spec for a grid container, from its stored config and its current inner bounds. */
export function gridSpecFor(config: WidgetConfigRecord | undefined, parent: WidgetLayout): GridLayoutSpec {
  const cfg = readLayoutGridConfig(config, Math.max(1, Math.floor(parent.w)), Math.max(1, Math.floor(parent.h)));
  const columns = cfg.autoSize
    ? Math.max(1, Math.min(96, Math.round(parent.w)))
    : Math.max(1, Math.floor(cfg.columns));
  const rows = cfg.autoSize
    ? Math.max(1, Math.min(96, Math.round(parent.h)))
    : Math.max(1, Math.floor(cfg.rows));
  return {
    parent,
    columns,
    rows,
    xEdges: buildGridAxisEdges(parent.x, parent.w, columns),
    yEdges: buildGridAxisEdges(parent.y, parent.h, rows)
  };
}

/** 1-based cell index whose edges contain `value` (binary search, clamped to the axis). */
export function axisCellAtValue(value: number, edges: number[]): number {
  const count = Math.max(1, edges.length - 1);
  if (value <= edges[0]) return 1;
  if (value >= edges[count]) return count;

  let low = 0;
  let high = count - 1;
  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    const cellStart = edges[mid];
    const cellEnd = edges[mid + 1];
    if (value < cellStart) {
      high = mid - 1;
    } else if (value >= cellEnd) {
      low = mid + 1;
    } else {
      return mid + 1;
    }
  }
  return Math.max(1, Math.min(count, low + 1));
}

/** Cell index and span covered by a continuous range along one axis. */
export function axisCellFromRange(
  rangeStart: number,
  rangeEnd: number,
  edges: number[]
): { index: number; span: number } {
  const count = Math.max(1, edges.length - 1);
  const axisMin = edges[0];
  const axisMax = edges[count];
  const start = Math.max(axisMin, Math.min(axisMax, Math.min(rangeStart, rangeEnd)));
  const end = Math.max(axisMin, Math.min(axisMax, Math.max(rangeStart, rangeEnd)));
  const axisSpan = Math.max(1e-6, axisMax - axisMin);
  const averageCellSpan = axisSpan / count;
  const epsilon = Math.max(1e-6, averageCellSpan * 0.01);

  const interiorStart = Math.max(axisMin, Math.min(axisMax, start + epsilon));
  const interiorEnd = Math.max(axisMin, Math.min(axisMax, end - epsilon));
  const sampleStart = interiorStart <= interiorEnd ? interiorStart : (start + end) * 0.5;
  const sampleEnd = interiorStart <= interiorEnd ? interiorEnd : sampleStart;
  const first = axisCellAtValue(sampleStart, edges);
  const last = axisCellAtValue(sampleEnd, edges);

  if (first > last) {
    const clamped = Math.max(1, Math.min(count, first));
    return { index: clamped, span: 1 };
  }

  return {
    index: first,
    span: Math.max(1, last - first + 1)
  };
}

/** Start and size, in grid units, of a cell range along one axis. */
export function axisRangeFromCell(
  index: number,
  span: number,
  edges: number[]
): { start: number; span: number } {
  const count = Math.max(1, edges.length - 1);
  const clampedIndex = Math.max(1, Math.min(count, Math.floor(index)));
  const maxSpan = count - clampedIndex + 1;
  const clampedSpan = Math.max(1, Math.min(maxSpan, Math.floor(span)));
  const start = edges[clampedIndex - 1];
  const end = edges[clampedIndex + clampedSpan - 1];
  return {
    start,
    span: Math.max(1e-6, end - start)
  };
}

/** Absolute layout of a grid-unit cell range (column/row counts, 1-based). */
export function gridCellFromAbsolute(spec: GridLayoutSpec, layout: WidgetLayout): WidgetLayout {
  const maxX = spec.parent.x + spec.parent.w;
  const maxY = spec.parent.y + spec.parent.h;
  const xStart = Math.max(spec.parent.x, Math.min(maxX, layout.x));
  const yStart = Math.max(spec.parent.y, Math.min(maxY, layout.y));
  const xEnd = Math.max(xStart, Math.min(maxX, layout.x + layout.w));
  const yEnd = Math.max(yStart, Math.min(maxY, layout.y + layout.h));

  const col = axisCellFromRange(xStart, xEnd, spec.xEdges);
  const row = axisCellFromRange(yStart, yEnd, spec.yEdges);
  return {
    x: col.index,
    y: row.index,
    w: Math.max(1, Math.min(spec.columns - col.index + 1, col.span)),
    h: Math.max(1, Math.min(spec.rows - row.index + 1, row.span))
  };
}

/** Absolute (grid-unit) layout for a cell range inside a grid container. */
export function absoluteFromGridCell(
  spec: GridLayoutSpec,
  cell: WidgetLayout,
  gridColumns: number
): WidgetLayout {
  const xRange = axisRangeFromCell(cell.x, cell.w, spec.xEdges);
  const yRange = axisRangeFromCell(cell.y, cell.h, spec.yEdges);
  const maxX = spec.parent.x + spec.parent.w;
  const maxY = spec.parent.y + spec.parent.h;
  const x = Math.max(spec.parent.x, Math.min(maxX, xRange.start));
  const y = Math.max(spec.parent.y, Math.min(maxY, yRange.start));
  const w = Math.max(1e-6, Math.min(maxX - x, xRange.span));
  const h = Math.max(1e-6, Math.min(maxY - y, yRange.span));
  return clampLayout({ x, y, w, h }, gridColumns);
}
