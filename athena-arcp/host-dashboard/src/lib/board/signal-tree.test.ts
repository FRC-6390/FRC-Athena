import { describe, expect, test } from 'bun:test';
import type { SignalDescriptor } from '../data/types';
import { buildSignalTree, signalsUnder } from './signal-tree';

function sig(path: string, mechanismPath: string, extra: Partial<SignalDescriptor> = {}): SignalDescriptor {
  const local = mechanismPath ? path.slice(mechanismPath.length + 1) : path;
  return {
    signalId: Math.floor(Math.random() * 1e9),
    path,
    type: 'f64',
    typeTag: 3,
    unit: '',
    role: 'value',
    writable: false,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0,
    mechanismPath,
    localPath: local,
    name: path.split('/').pop() ?? path,
    ...extra
  };
}

const SIGNALS = [
  sig('robot/turret/Controls/fieldHeading/State/FeedbackPosition', 'robot/turret'),
  sig('robot/turret/Controls/fieldHeading/Config/PID/p', 'robot/turret'),
  sig('robot/turret/Devices/motor/State/SupplyCurrentAmps', 'robot/turret'),
  sig('robot/turret/Actions/left', 'robot/turret', { type: 'bool' }),
  sig('robot/turret/State/Reason', 'robot/turret', { type: 'string' }),
  sig('athena/power/battery/voltage', '', { role: 'system' }),
  sig('athena/locks/Intake', '', { role: 'lock' })
];

describe('signal tree', () => {
  test('mechanism, groups in a useful order, controls and devices as units', () => {
    const [turret] = buildSignalTree(SIGNALS);
    expect(turret.label).toBe('Turret');
    expect(turret.unit).toBe('mechanism');
    expect(turret.count).toBe(5);
    expect(turret.children.map((c) => c.label)).toEqual(['State', 'Actions', 'Controls', 'Devices']);
    const control = turret.children.find((c) => c.label === 'Controls')!.children[0];
    expect([control.label, control.unit, control.count]).toEqual(['Field heading', 'control', 2]);
    const device = turret.children.find((c) => c.label === 'Devices')!.children[0];
    expect([device.label, device.unit]).toEqual(['Motor', 'device']);
    expect(signalsUnder(device).map((s) => s.name)).toEqual(['SupplyCurrentAmps']);
  });

  test('system and lock signals are left out; the query filters by path', () => {
    expect(buildSignalTree(SIGNALS).map((r) => r.label)).toEqual(['Turret']);
    const [turret] = buildSignalTree(SIGNALS, 'pid');
    expect(turret.count).toBe(1);
  });
});
