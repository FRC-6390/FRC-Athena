import { WidgetConfigRecord } from '../dashboard';
import { isObject, looksLikeColor, toBoolean, toNumber, toString } from './common';

export type LayoutTitleConfig = {
  text: string;
  color: string;
};

export type LayoutGridConfig = {
  columns: number;
  rows: number;
  autoSize: boolean;
};

export type LayoutAccordionConfig = {
  collapsed: boolean;
  expandedRows: number | null;
};

export function readLayoutTitleConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackText: string
): LayoutTitleConfig {
  const base = isObject(raw) ? raw : {};
  const text = toString(base.text, fallbackText).trim() || fallbackText;
  const colorRaw = toString(base.color, '#f5f7fb').trim();
  const color = looksLikeColor(colorRaw) ? colorRaw : '#f5f7fb';
  return { text, color };
}

export function readLayoutGridConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackColumns: number,
  fallbackRows: number
): LayoutGridConfig {
  const base = isObject(raw) ? raw : {};
  const columns = Math.max(1, Math.min(96, Math.floor(toNumber(base.columns) ?? fallbackColumns)));
  const rows = Math.max(1, Math.min(96, Math.floor(toNumber(base.rows) ?? fallbackRows)));
  const autoSize = toBoolean(base.autoSize, true);
  return { columns, rows, autoSize };
}

export function readLayoutAccordionConfig(
  raw: WidgetConfigRecord | undefined
): LayoutAccordionConfig {
  const base = isObject(raw) ? raw : {};
  const collapsed = toBoolean(base.collapsed, false);
  const expandedRowsRaw = toNumber(base.expandedRows);
  const expandedRows =
    expandedRowsRaw === null
      ? null
      : Math.max(1, Math.min(96, Math.round(expandedRowsRaw)));
  return { collapsed, expandedRows };
}
