import { describe, expect, test } from 'bun:test';
import {
  accordionStackLayer,
  hiddenByCollapsedAccordion,
  renderOrder,
  toggledAccordionConfig,
  wouldCreateParentCycle
} from './tree';
import { at, widget } from './test-support';

const byIdOf = (widgets: ReturnType<typeof widget>[]) => new Map(widgets.map((w) => [w.id, w] as const));

describe('render order', () => {
  test('containers come before their children and children follow their parent', () => {
    const widgets = [
      widget('metric-b', 'metric', at(5, 1, 2, 1)),
      widget('child', 'metric', at(1, 1, 2, 1), { parentLayoutId: 'list' }),
      widget('list', 'layout_list', at(1, 2, 4, 3)),
      widget('title', 'layout_title', at(1, 1, 4, 1))
    ];
    expect(renderOrder(widgets, byIdOf(widgets)).map((w) => w.id)).toEqual([
      'title',
      'list',
      'child',
      'metric-b'
    ]);
  });

  test('a child whose parent is missing is still drawn', () => {
    const widgets = [widget('orphan', 'metric', at(1, 1, 2, 1), { parentLayoutId: 'gone' })];
    expect(renderOrder(widgets, byIdOf(widgets)).map((w) => w.id)).toEqual(['orphan']);
  });
});

describe('accordions', () => {
  const accordion = (id: string, collapsed: boolean) =>
    widget(id, 'layout_accordion', at(1, 1, 4, 3), { config: { collapsed } });

  test('children of a collapsed accordion are hidden, including nested ones', () => {
    const widgets = [
      accordion('acc', true),
      widget('inner-list', 'layout_list', at(1, 1, 4, 3), { parentLayoutId: 'acc' }),
      widget('deep', 'metric', at(1, 1, 2, 1), { parentLayoutId: 'inner-list' }),
      widget('open', 'metric', at(1, 4, 2, 1))
    ];
    const hidden = hiddenByCollapsedAccordion(widgets, byIdOf(widgets));
    expect(hidden.get('inner-list')).toBe(true);
    expect(hidden.get('deep')).toBe(true);
    expect(hidden.get('open')).toBe(false);
  });

  test('stack layer grows with each expanded accordion in the chain', () => {
    const widgets = [
      accordion('outer', false),
      widget('inner', 'layout_accordion', at(1, 1, 4, 3), { parentLayoutId: 'outer', config: { collapsed: false } }),
      widget('leaf', 'metric', at(1, 1, 2, 1), { parentLayoutId: 'inner' })
    ];
    const byId = byIdOf(widgets);
    // Both accordions are expanded: the inner shell adds 2 as the first element, the outer adds 3.
    expect(accordionStackLayer(byId, widgets[2])).toBe(3 + 3);
    expect(accordionStackLayer(byId, widgets[1])).toBe(2 + 3);
  });

  test('toggling keeps every other config key', () => {
    const next = toggledAccordionConfig(widget('acc', 'layout_accordion', at(1, 1, 4, 3), { config: { collapsed: false, expandedRows: 4 } }));
    expect(next).toEqual({ collapsed: true, expandedRows: 4 });
  });
});

describe('parent cycles', () => {
  test('a widget cannot become a descendant of itself', () => {
    const widgets = [
      widget('a', 'layout_list', at(1, 1, 4, 3)),
      widget('b', 'layout_list', at(1, 4, 4, 3), { parentLayoutId: 'a' })
    ];
    const byId = byIdOf(widgets);
    expect(wouldCreateParentCycle(byId, 'a', 'b')).toBe(true);
    expect(wouldCreateParentCycle(byId, 'b', 'a')).toBe(false);
  });
});
