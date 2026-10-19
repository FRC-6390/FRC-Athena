import { describe, expect, test } from 'bun:test';
import { createSession } from '../data/session';
import { createMockSource } from '../data/mock';
import { buildMechanismTree, flattenMechanisms } from '../data/tree';
import { MOCK_SPECS } from '../data/mock-robot';
import { controlSections, deviceHealth, deviceSections, summarize } from './model';
import { describePath } from '../data/path';
import { TYPE_TAG, type SignalDescriptor } from '../data/types';

/** Builds the dictionary of the mock robot without starting any timers. */
function mockDictionary(): SignalDescriptor[] {
  return MOCK_SPECS.map((spec, index) => ({
    signalId: index + 1,
    ...describePath(spec.path),
    type: spec.type,
    typeTag: spec.typeTag,
    unit: spec.unit,
    role: spec.role,
    writable: spec.writable,
    persisted: spec.persisted,
    lockGroup: spec.lockGroup,
    structSchemaId: 0
  }));
}

describe('mechanism model on the mock robot', () => {
  const roots = buildMechanismTree(mockDictionary());
  const byPath = new Map(flattenMechanisms(roots).map((n) => [n.path, n]));

  test('turret page splits controls into feedback, gains and constraints', () => {
    const turret = byPath.get('robot/turret')!;
    const [position] = controlSections(turret);
    expect(position.name).toBe('position');
    expect(position.plot.map((s) => s.name)).toEqual(['FeedbackPosition', 'FeedbackVelocity']);
    expect(position.gains.map((s) => s.name)).toEqual(['p', 'i', 'd', 'static', 'velocity']);
    expect(position.constraints.map((s) => s.name)).toEqual([
      'MaximumAcceleration',
      'MaximumPosition',
      'MaximumVelocity',
      'MinimumPosition'
    ]);
  });

  test('devices group by device name, and the motor current is the supply current', () => {
    const turret = byPath.get('robot/turret')!;
    const sections = deviceSections(turret);
    expect(sections.map((d) => d.name)).toEqual(['encoder', 'motor']);
    const motor = sections.find((d) => d.name === 'motor')!;
    expect(motor.current?.name).toBe('SupplyCurrentAmps');
    expect(motor.temperature).toBeNull();
    expect(motor.derate).toBeNull();
  });

  test('summary finds the reason, active action and fault; there is no stage signal', () => {
    const turret = byPath.get('robot/turret')!;
    const summary = summarize(turret);
    expect(summary.state?.name).toBe('ActiveAction');
    expect(summary.reason?.name).toBe('Reason');
    expect(summary.fault?.name).toBe('Fault');
    expect(summary.running?.name).toBe('ActionRunning');
    expect(summary.stage).toBeNull();
    expect(summary.writableCount).toBeGreaterThan(0);
  });

  test('child mechanisms keep their own sections', () => {
    const hood = byPath.get('robot/turret/hood')!;
    expect(hood.signals.every((s) => s.mechanismPath === 'robot/turret/hood')).toBe(true);
    expect(summarize(hood).state?.name).toBe('ActiveAction');
  });

  test('device health hints read temperature and derate by their real names', () => {
    const named = (name: string): SignalDescriptor => ({
      signalId: 0,
      ...describePath(`robot/x/Devices/motor/State/${name}`),
      type: 'f64',
      typeTag: TYPE_TAG.f64,
      unit: '',
      role: 'device',
      writable: false,
      persisted: false,
      lockGroup: '',
      structSchemaId: 0
    });
    const temp = named('TemperatureCelsius');
    expect(deviceHealth(temp, 75)).toBe('warn');
    expect(deviceHealth(temp, 40)).toBe('ok');
    const derate = named('Derate');
    expect(deviceHealth(derate, 0.7)).toBe('derate');
    expect(deviceHealth(derate, 1)).toBe('ok');
  });
});

describe('live session smoke test', () => {
  test('the mock session fills the dictionary so pages can render', async () => {
    const session = createSession(createMockSource({ tickMs: 5 }));
    session.start();
    await new Promise((resolve) => setTimeout(resolve, 60));
    const roots = buildMechanismTree(session.store.allSignals());
    expect(flattenMechanisms(roots).map((n) => n.path)).toEqual([
      'robot',
      'robot/drive',
      'robot/shooter',
      'robot/turret',
      'robot/turret/hood'
    ]);
    session.stop();
  });
});
