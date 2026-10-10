/**
 * The mechanism helpers run against the simulated robot's real dictionary (fixtures/real-dictionary.json), so
 * they are proven against the names the robot publishes, not names invented for the mock.
 */
import { describe, expect, test } from 'bun:test';
import { buildMechanismTree, flattenMechanisms, groupSignals, withinGroup, type MechanismNode } from '../data/tree';
import { MOCK_SPECS } from '../data/mock-robot';
import { describePath } from '../data/path';
import { realDictionary } from '../data/fixtures/real-dictionary';
import { TYPE_TAG, type SignalDescriptor } from '../data/types';
import { actionSignals, controlSections, deviceHealth, deviceSections, GAIN_NAMES, summarize } from './model';

const real = realDictionary();
const nodes = flattenMechanisms(buildMechanismTree(real));
const node = (path: string): MechanismNode => nodes.find((n) => n.path === path)!;

/** Shape of a signal below its mechanism: device, control, action and value names collapse to `*`. */
function shapeOf(signal: SignalDescriptor): string {
  const [group, ...rest] = signal.localPath.split('/');
  if (group === 'Devices' || group === 'Controls') return [group, '*', ...rest.slice(1)].join('/');
  if (group === 'Actions' || group === 'Values') return `${group}/*`;
  return signal.localPath;
}

describe('real dictionary: summaries', () => {
  test('every mechanism with state publishes reason, active action and fault under State/', () => {
    const stateful = nodes.filter((n) => groupSignals(n).State.length > 0);
    expect(stateful.length).toBeGreaterThan(10);
    for (const n of stateful) {
      const summary = summarize(n);
      expect([n.path, summary.reason?.localPath]).toEqual([n.path, 'State/Reason']);
      expect([n.path, summary.state?.localPath]).toEqual([n.path, 'State/ActiveAction']);
      expect([n.path, summary.fault?.localPath]).toEqual([n.path, 'State/Fault']);
      expect(summary.stage).toBeNull();
    }
  });

  test('turret summary names the real signals and counts writable ones', () => {
    const summary = summarize(node('robot/turret'));
    expect(summary.reason?.path).toBe('robot/turret/State/Reason');
    expect(summary.running?.path).toBe('robot/turret/State/ActionRunning');
    expect(summary.writableCount).toBeGreaterThan(0);
  });

  test('the real robot publishes no stage signal', () => {
    expect(real.some((s) => s.localPath.endsWith('State/stage'))).toBe(false);
  });

  test('system and lock signals stay out of the mechanism tree', () => {
    expect(nodes.some((n) => n.path === 'athena' || n.path.startsWith('athena/'))).toBe(false);
  });
});

describe('real dictionary: controls', () => {
  test('feedback traces are FeedbackPosition and FeedbackVelocity; there are no setpoint traces', () => {
    const [fieldHeading] = controlSections(node('robot/turret'));
    expect(fieldHeading.name).toBe('fieldHeading');
    expect(fieldHeading.plot.map((s) => s.name)).toEqual(['FeedbackPosition', 'FeedbackVelocity']);
    expect(real.some((s) => /Controls\/[^/]+\/(target|reference|measured)$/.test(s.localPath))).toBe(false);
  });

  test('PID gains are gains, the PID disabled flag and config flags are not', () => {
    const [fieldHeading] = controlSections(node('robot/turret'));
    expect(fieldHeading.gains.map((s) => s.name)).toEqual(['p', 'i', 'd', 'iZone']);
    expect(fieldHeading.gains.every((s) => s.writable)).toBe(true);
    expect(fieldHeading.other.map((s) => s.name).sort()).toEqual([
      'Disabled',
      'Mode',
      'Motors',
      'Percent',
      'Restore',
      'RunPercent',
      'Slot',
      'Type',
      'disabled'
    ]);
  });

  test('feedforward gains are kS, kV, kA and kG, and constraints are the four Maximum/Minimum limits', () => {
    const wheel = controlSections(node('robot/shooter')).find((c) => c.name === 'wheel')!;
    expect(wheel.gains.map((s) => s.name)).toEqual([
      'p',
      'i',
      'd',
      'iZone',
      'static',
      'velocity',
      'acceleration',
      'gravity'
    ]);
    const shoulder = controlSections(node('robot/arm')).find((c) => c.name === 'shoulder')!;
    expect(shoulder.constraints.map((s) => s.name)).toEqual([
      'MaximumAcceleration',
      'MaximumPosition',
      'MaximumVelocity',
      'MinimumPosition'
    ]);
  });

  test('every real PID or feedforward leaf is either a known gain or the disabled flag', () => {
    const gainLeaves = real
      .map((s) => /Config\/(PID|Feedforward)\/(.+)$/.exec(s.localPath))
      .filter((m): m is RegExpExecArray => m !== null)
      .map((m) => `${m[1]}/${m[2]}`);
    expect(gainLeaves.length).toBeGreaterThan(0);
    for (const leaf of new Set(gainLeaves)) {
      expect([leaf, leaf.endsWith('/disabled') || leaf in GAIN_NAMES]).toEqual([leaf, true]);
    }
  });

  test('gain names map to the usual kP, kI, kD, kIZone, kS, kV, kA and kG', () => {
    expect(Object.values(GAIN_NAMES)).toEqual(['kP', 'kI', 'kD', 'kIZone', 'kS', 'kV', 'kA', 'kG']);
  });
});

describe('real dictionary: devices and actions', () => {
  test('device sections group by device; the motor current is the supply current', () => {
    const sections = deviceSections(node('robot/turret'));
    expect(sections.map((d) => d.name)).toEqual(['angle', 'chassisImu', 'motor']);
    const motor = sections.find((d) => d.name === 'motor')!;
    expect(motor.current?.path).toBe('robot/turret/Devices/motor/State/SupplyCurrentAmps');
    expect(motor.temperature).toBeNull();
    expect(motor.derate).toBeNull();
    expect(sections.find((d) => d.name === 'angle')!.current).toBeNull();
  });

  test('temperature and derate are not published over ARCP, so health stays ok on the real robot', () => {
    expect(real.some((s) => s.name === 'TemperatureCelsius' || s.name === 'Derate')).toBe(false);
    const temp = { ...describePath('robot/x/Devices/motor/State/TemperatureCelsius'), signalId: 1, type: 'f64', typeTag: TYPE_TAG.f64, unit: '', role: 'device', writable: false, persisted: false, lockGroup: '', structSchemaId: 0 } as SignalDescriptor;
    expect(deviceHealth(temp, 80)).toBe('warn');
  });

  test('actions are the bool action signals of a mechanism, nested ones included', () => {
    const names = actionSignals(node('robot/turret')).map((s) => s.name);
    expect(names).toEqual(expect.arrayContaining(['forward', 'left', 'neutral', 'rear', 'DynamicForward']));
    expect(actionSignals(node('robot/turret')).every((s) => s.type === 'bool')).toBe(true);
  });

  test('the mock publishes only shapes the real robot publishes', () => {
    const realShapes = new Set(real.filter((s) => s.mechanismPath.startsWith('robot/')).map(shapeOf));
    const mockShapes = MOCK_SPECS.filter((spec) => spec.path.startsWith('robot/')).map((spec) => {
      const { localPath, mechanismPath } = describePath(spec.path);
      const signal = { localPath, mechanismPath } as SignalDescriptor;
      return { path: spec.path, shape: shapeOf(signal) };
    });
    const unknown = mockShapes.filter((m) => !realShapes.has(m.shape));
    expect(unknown).toEqual([]);
  });
});
