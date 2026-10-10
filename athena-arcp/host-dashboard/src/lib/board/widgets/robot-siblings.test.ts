import { describe, expect, test } from 'bun:test';
import type { SignalDescriptor } from '../../data/types';
import { controlRoot, controlSiblings, deviceSiblings, devicePrefix, mechanismSiblings, normalizeName } from './robot-siblings';
import { powerBreakdown } from './robot-power';

function sig(path: string, extra: Partial<SignalDescriptor> = {}): SignalDescriptor {
  const segments = path.split('/');
  const groupIndex = segments.findIndex((s) => ['State', 'Values', 'Actions', 'Devices', 'Controls', 'Hooks', 'Trace'].includes(s));
  const mechanismPath = groupIndex < 0 ? segments.slice(0, -1).join('/') : segments.slice(0, groupIndex).join('/');
  return {
    signalId: Math.floor(Math.random() * 1e9),
    path,
    type: 'f64',
    typeTag: 3,
    unit: '',
    role: 'device',
    writable: false,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0,
    mechanismPath,
    localPath: groupIndex < 0 ? segments[segments.length - 1] : segments.slice(groupIndex).join('/'),
    name: segments[segments.length - 1],
    ...extra
  };
}

const ALL = [
  sig('robot/turret/Devices/motor/current', { unit: 'A' }),
  sig('robot/turret/Devices/motor/temperature'),
  sig('robot/turret/Devices/motor/derate'),
  sig('robot/turret/Devices/motor/Applied Voltage'),
  sig('robot/turret/Devices/motor/connected', { type: 'bool' }),
  sig('robot/turret/Devices/encoder/position'),
  sig('robot/turret/State/state', { type: 'string' }),
  sig('robot/turret/State/reason', { type: 'string' }),
  sig('robot/turret/State/FaultLatched', { type: 'bool' }),
  sig('robot/turret/Actions/home', { type: 'bool', role: 'action' }),
  sig('robot/turret/hood/State/state', { type: 'string' }),
  sig('robot/turret/Controls/position/target'),
  sig('robot/turret/Controls/position/reference'),
  sig('robot/turret/Controls/position/measured'),
  sig('robot/turret/Controls/position/kP'),
  sig('robot/turret/Controls/position/kD'),
  sig('robot/turret/Controls/position/Constraints/maxVelocity'),
  sig('robot/turret/Controls/fieldHeading/State/FeedbackPosition'),
  sig('robot/turret/Controls/fieldHeading/Config/PID/p'),
  sig('robot/turret/Controls/fieldHeading/Config/PID/i')
];

describe('device siblings', () => {
  test('prefix and lookups by name', () => {
    expect(devicePrefix('robot/turret/Devices/motor/current')).toBe('robot/turret/Devices/motor');
    expect(devicePrefix('robot/turret/State/state')).toBeNull();
    expect(devicePrefix('robot/turret/Devices/motor')).toBeNull();
    const d = deviceSiblings(ALL, 'robot/turret/Devices/motor/current');
    expect(d!.name).toBe('motor');
    expect(d!.current?.path).toBe('robot/turret/Devices/motor/current');
    expect(d!.voltage?.name).toBe('Applied Voltage');
    expect(d!.connected?.name).toBe('connected');
    expect(d!.children.length).toBe(5);
  });
  test('normalizeName ignores case and punctuation', () => {
    expect(normalizeName('Applied_Voltage')).toBe('appliedvoltage');
  });
});

describe('mechanism siblings', () => {
  test('own State, faults and actions only', () => {
    const m = mechanismSiblings(ALL, 'robot/turret');
    expect(m.state?.path).toBe('robot/turret/State/state');
    expect(m.reason?.path).toBe('robot/turret/State/reason');
    expect(m.stage).toBeNull();
    expect(m.faults.map((s) => s.name)).toEqual(['FaultLatched']);
    expect(m.actions.map((s) => s.name)).toEqual(['home']);
  });
});

describe('control siblings', () => {
  test('root and traces for a standard loop', () => {
    expect(controlRoot('robot/turret/Controls/position/target')).toBe('robot/turret/Controls/position');
    expect(controlRoot('robot/turret/State/state')).toBeNull();
    const c = controlSiblings(ALL, 'robot/turret/Controls/position/kP');
    expect(c!.target?.name).toBe('target');
    expect(c!.reference?.name).toBe('reference');
    expect(c!.measured?.name).toBe('measured');
    expect(c!.gains.map((s) => s.name)).toEqual(['kD', 'kP']);
  });
  test('feedback names and PID config gains', () => {
    const c = controlSiblings(ALL, 'robot/turret/Controls/fieldHeading/Config/PID/p');
    expect(c!.measured?.path).toBe('robot/turret/Controls/fieldHeading/State/FeedbackPosition');
    expect(c!.target).toBeNull();
    expect(c!.gains.map((s) => s.name)).toEqual(['i', 'p']);
  });
});

describe('power breakdown', () => {
  test('signed total, magnitude fractions, non-finite as zero', () => {
    const b = powerBreakdown([
      { label: 'a', amps: 30 },
      { label: 'b', amps: -10 },
      { label: 'c', amps: Number.NaN }
    ]);
    expect(b.total).toBe(20);
    expect(b.magnitude).toBe(40);
    expect(b.shares[0].fraction).toBeCloseTo(0.75);
    expect(b.shares[2].fraction).toBe(0);
    expect(powerBreakdown([{ label: 'x', amps: 0 }]).shares[0].fraction).toBe(0);
  });
});
