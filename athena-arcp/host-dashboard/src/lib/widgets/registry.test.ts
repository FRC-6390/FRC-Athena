import { describe, expect, test } from 'bun:test';
import { describePath } from '../data/path';
import type { SignalDescriptor, SignalRole, SignalType } from '../data/types';
import { TYPE_TAG } from '../data/types';
import { classicKindsFor, compatibleModules, WIDGET_MODULES, widgetModule } from './registry';
import { SIGNAL_WIDGET_KINDS } from './kinds';

function signal(
  path: string,
  type: SignalType,
  options: { role?: SignalRole; writable?: boolean } = {}
): SignalDescriptor {
  return {
    signalId: 1,
    ...describePath(path),
    type,
    typeTag: TYPE_TAG.f64,
    unit: '',
    role: options.role ?? 'value',
    writable: options.writable ?? false,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0
  };
}

const ids = (s: SignalDescriptor) => compatibleModules(s).map((m) => m.id);

describe('widget registry compatibility', () => {
  test('read-only numeric signals offer graph-like widgets and never write widgets', () => {
    const offered = ids(signal('robot/turret/Controls/position/measured', 'f64'));
    expect(offered).toEqual(expect.arrayContaining(['metric', 'graph', 'plot', 'dial']));
    expect(offered).not.toContain('input');
    expect(offered).not.toContain('tunable');
  });

  test('writable numeric signals offer input and tunable', () => {
    const offered = ids(signal('robot/turret/Values/target', 'f64', { writable: true, role: 'value' }));
    expect(offered).toEqual(expect.arrayContaining(['input', 'tunable', 'dropdown']));
  });

  test('read-only bool offers status matrix, not toggle', () => {
    const offered = ids(signal('robot/turret/State/atSetpoint', 'bool', { role: 'state' }));
    expect(offered).toEqual(expect.arrayContaining(['status_matrix', 'state', 'metric']));
    expect(offered).not.toContain('toggle');
  });

  test('action signals offer only action, state and text', () => {
    const offered = ids(signal('robot/turret/Actions/home', 'bool', { role: 'action' }));
    expect([...offered].sort()).toEqual(['action', 'state', 'text']);
  });

  test('numeric arrays offer pose-style widgets, not graphs', () => {
    const offered = ids(signal('robot/drive/State/moduleStates', 'f64[]'));
    expect(offered).toEqual(expect.arrayContaining(['field', 'imu_3d', 'swerve_drive', 'text']));
    expect(offered).not.toContain('graph');
  });

  test('hardware widgets require their path hint', () => {
    expect(ids(signal('robot/shooter/Hooks/ready', 'bool'))).not.toContain('motor');
  });

  test('the registry resolves plot by id and exposes its config reader', () => {
    const plot = widgetModule('plot');
    expect(plot?.readConfig?.(undefined)).toEqual({ windowSeconds: 20, yMin: null, yMax: null });
    expect(plot?.readConfig?.({ windowSeconds: 9999, yMin: 'x' })).toEqual({
      windowSeconds: 300,
      yMin: null,
      yMax: null
    });
  });
});

describe('classic kinds are registered once', () => {
  test('every saved classic kind has exactly one legacy module', () => {
    const legacyIds = WIDGET_MODULES.filter((m) => m.legacy).map((m) => m.id);
    expect([...legacyIds].sort()).toEqual([...SIGNAL_WIDGET_KINDS].sort());
    expect(new Set(legacyIds).size).toBe(legacyIds.length);
  });

  test('new-contract modules never appear in the classic palette', () => {
    expect(classicKindsFor(null)).not.toContain('plot' as never);
    expect(classicKindsFor(null).length).toBeGreaterThan(0);
  });
});
