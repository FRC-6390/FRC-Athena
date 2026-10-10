/**
 * Finds the signals that belong with one dropped signal: the motor's other device readings, the mechanism's
 * State and actions, or the control loop's traces and gains. Pure lookups over the store's dictionary.
 */
import type { SignalDescriptor } from '../../data/types';
import { isGainName } from '../../mechanisms/model';

/** Lower-case letters only, so `Applied Voltage`, `appliedVoltage` and `applied_voltage` compare equal. */
export function normalizeName(name: string): string {
  return name.toLowerCase().replace(/[^a-z]/g, '');
}

/** Device prefix of a `.../Devices/<device>/...` path, e.g. `robot/turret/Devices/motor`. Null when not a device. */
export function devicePrefix(path: string): string | null {
  const parts = path.split('/');
  const at = parts.lastIndexOf('Devices');
  // Needs a device name after `Devices` and a leaf signal after the device name.
  if (at < 0 || at + 2 > parts.length - 1) return null;
  return parts.slice(0, at + 2).join('/');
}

export type DeviceSiblings = {
  prefix: string;
  name: string;
  current: SignalDescriptor | null;
  temperature: SignalDescriptor | null;
  derate: SignalDescriptor | null;
  voltage: SignalDescriptor | null;
  connected: SignalDescriptor | null;
  /** Every direct child of the device, sorted by name. */
  children: SignalDescriptor[];
};

const DEVICE_NAMES = {
  current: ['current', 'statorcurrent', 'supplycurrent', 'amps'],
  temperature: ['temperature', 'temp', 'tempc'],
  derate: ['derate', 'derating'],
  voltage: ['appliedvoltage', 'voltage', 'volts'],
  connected: ['connected', 'isconnected', 'online']
} as const;

/** Sibling readings of a device, found by path prefix. Null when the anchor is not under `Devices/`. */
export function deviceSiblings(all: readonly SignalDescriptor[], anchorPath: string): DeviceSiblings | null {
  const prefix = devicePrefix(anchorPath);
  if (prefix === null) return null;
  const children = all
    .filter((s) => s.path.startsWith(prefix + '/') && !s.path.slice(prefix.length + 1).includes('/'))
    .sort((a, b) => a.name.localeCompare(b.name));
  const pick = (candidates: readonly string[]) =>
    candidates.map((c) => children.find((s) => normalizeName(s.name) === c)).find((s) => s !== undefined) ?? null;
  return {
    prefix,
    name: prefix.split('/').pop() ?? prefix,
    current: pick(DEVICE_NAMES.current),
    temperature: pick(DEVICE_NAMES.temperature),
    derate: pick(DEVICE_NAMES.derate),
    voltage: pick(DEVICE_NAMES.voltage),
    connected: pick(DEVICE_NAMES.connected),
    children
  };
}

export type MechanismSiblings = {
  mechanismPath: string;
  state: SignalDescriptor | null;
  reason: SignalDescriptor | null;
  stage: SignalDescriptor | null;
  /** Signals whose name marks a fault (boolean or otherwise). */
  faults: SignalDescriptor[];
  /** Boolean action signals of this mechanism. */
  actions: SignalDescriptor[];
};

/** The mechanism's own State, fault and action signals. Child mechanisms are not included. */
export function mechanismSiblings(all: readonly SignalDescriptor[], mechanismPath: string): MechanismSiblings {
  const own = all.filter((s) => s.mechanismPath === mechanismPath);
  const local = (name: string) => own.find((s) => s.localPath === name) ?? null;
  return {
    mechanismPath,
    state: local('State/state'),
    reason: local('State/reason'),
    stage: local('State/stage'),
    faults: own.filter((s) => s.localPath.startsWith('State/') && /fault/i.test(s.name)),
    actions: own.filter((s) => s.localPath.startsWith('Actions/') && s.type === 'bool')
  };
}

export type ControlSiblings = {
  root: string;
  name: string;
  target: SignalDescriptor | null;
  reference: SignalDescriptor | null;
  measured: SignalDescriptor | null;
  velocity: SignalDescriptor | null;
  /** Gains sorted by name: `kP`, `kD`, ... and `Config/PID/p`-style entries. */
  gains: SignalDescriptor[];
};

/** The `.../Controls/<name>` prefix of a control signal, or null when the path is not under a control. */
export function controlRoot(path: string): string | null {
  const parts = path.split('/');
  const at = parts.lastIndexOf('Controls');
  if (at < 0 || at + 2 > parts.length - 1) return null;
  return parts.slice(0, at + 2).join('/');
}

const TARGET_LOCAL = ['target', 'Target', 'setpoint', 'Setpoint'];
const REFERENCE_LOCAL = ['reference', 'Reference'];
const MEASURED_LOCAL = ['measured', 'Measured', 'State/FeedbackPosition'];
const VELOCITY_LOCAL = ['State/FeedbackVelocity', 'measuredVelocity'];

function isGain(within: string): boolean {
  if (within.includes('/')) {
    const parts = within.split('/');
    return parts.length === 3 && parts[0] === 'Config' && parts[1] === 'PID' && /^[pidPID]$/.test(parts[2]);
  }
  return isGainName(within);
}

/** Control loop siblings of a control signal. Null when the path is not under `Controls/`. */
export function controlSiblings(all: readonly SignalDescriptor[], anchorPath: string): ControlSiblings | null {
  const root = controlRoot(anchorPath);
  if (root === null) return null;
  const under = all.filter((s) => s.path.startsWith(root + '/'));
  const within = (s: SignalDescriptor) => s.path.slice(root.length + 1);
  const pick = (names: readonly string[]) =>
    names.map((n) => under.find((s) => within(s) === n)).find((s) => s !== undefined) ?? null;
  return {
    root,
    name: root.split('/').pop() ?? root,
    target: pick(TARGET_LOCAL),
    reference: pick(REFERENCE_LOCAL),
    measured: pick(MEASURED_LOCAL),
    velocity: pick(VELOCITY_LOCAL),
    gains: under.filter((s) => isGain(within(s))).sort((a, b) => a.name.localeCompare(b.name))
  };
}
