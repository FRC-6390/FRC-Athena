import { beforeEach, describe, expect, test } from 'bun:test';
import layoutFixture from './fixtures/layout-v2-classic.json';
import exportFixture from './fixtures/layout-export-classic.json';
import {
  loadLayout,
  normalizeLayoutState,
  parseLayoutImport,
  saveLayout,
  serializeLayoutExport,
  type DashboardLayoutState
} from '../dashboard';

// Golden fixtures were produced by the pre-split (monolithic WidgetCanvas era) code. Saved layouts in
// the field must keep loading unchanged, so these assertions pin the on-disk / export format.

const layout = layoutFixture as DashboardLayoutState;

function memoryStorage() {
  const store = new Map<string, string>();
  return {
    getItem: (key: string) => store.get(key) ?? null,
    setItem: (key: string, value: string) => void store.set(key, value),
    removeItem: (key: string) => void store.delete(key)
  };
}

describe('saved layout format (golden fixture from pre-split code)', () => {
  beforeEach(() => {
    (globalThis as { localStorage?: unknown }).localStorage = memoryStorage();
  });

  test('normalizeLayoutState accepts the fixture unchanged', () => {
    expect(normalizeLayoutState(layoutFixture)).toEqual(layout);
  });

  test('every widget kind in the fixture survives normalization with its config', () => {
    const widgets = layout.tabs.flatMap((tab) => tab.widgets);
    const kinds = new Set(widgets.map((widget) => widget.kind));
    expect(kinds.size).toBeGreaterThanOrEqual(31);
    expect(widgets.find((widget) => widget.kind === 'graph')?.config).toEqual({
      showLegend: true,
      yMin: -1,
      yMax: 1,
      series: [{ signalId: 1, style: 'line', color: '#ff0000' }]
    });
    expect(widgets.find((widget) => widget.id === 'widget-in-acc')?.parentLayoutId).toBeDefined();
  });

  test('export file imports to the same layout and re-exports in the same envelope', () => {
    expect(parseLayoutImport(JSON.stringify(exportFixture))).toEqual(layout);
    const reexported = JSON.parse(serializeLayoutExport(layout)) as Record<string, unknown>;
    expect(reexported.format).toBe(exportFixture.format);
    expect(reexported.version).toBe(exportFixture.version);
    expect(reexported.layout).toEqual(exportFixture.layout);
  });

  test('loadLayout reads the v2 storage key written by saveLayout', () => {
    localStorage.setItem('arcp.host.layout.v2', JSON.stringify(layoutFixture));
    expect(loadLayout()).toEqual(layout);
    saveLayout(layout);
    expect(JSON.parse(localStorage.getItem('arcp.host.layout.v2') ?? 'null')).toEqual(layoutFixture);
  });
});
