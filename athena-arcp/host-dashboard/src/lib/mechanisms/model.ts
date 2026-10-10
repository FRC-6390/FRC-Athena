/**
 * Mechanism signal conventions, as the Athena robot publishes them over ARCP. Checked against the simulated
 * robot's real dictionary (`src/lib/data/fixtures/real-dictionary.json`, 928 signals from example-projects/
 * mechanism-examples). A mechanism path is every segment before the first group segment, so
 * `robot/turret/hood/Values/target` belongs to `robot/turret/hood`.
 *
 * | Path (under a mechanism)                    | Type / role     | Meaning                                              |
 * |---------------------------------------------|-----------------|------------------------------------------------------|
 * | State/Reason                                | string / state  | Why the mechanism is in its state ("Disabled", ...)  |
 * | State/ActiveAction                          | string / state  | Running action name, empty when idle                 |
 * | State/ActionRunning, State/ActionComplete   | bool / state    | Action progress                                      |
 * | State/Fault                                 | string / state  | Fault text, empty when none                          |
 * | Actions/<name>, Actions/<group>/<name>      | bool / action   | Request an action (read-only; run through commands)  |
 * | Values/<name>                               | f64, bool, string, f64[] | Writable values are role control; read-only are role value |
 * | Controls/<name>/State/FeedbackPosition      | f64 / control   | Measured position (the plotted trace)                |
 * | Controls/<name>/State/FeedbackVelocity      | f64 / control   | Measured velocity (the plotted trace)                |
 * | Controls/<name>/State/Mode                  | string / control| Control mode, e.g. POSITION, VELOCITY                |
 * | Controls/<name>/Config/PID/p, i, d, iZone   | f64 / control, writable | PID gains kP, kI, kD, kIZone               |
 * | Controls/<name>/Config/Feedforward/static   | f64 / control, writable | kS                                         |
 * | Controls/<name>/Config/Feedforward/velocity, acceleration, gravity | f64 | kV, kA, kG                 |
 * | Controls/<name>/Config/Constraints/MinimumPosition, MaximumPosition, MaximumVelocity, MaximumAcceleration | f64, writable | Limits |
 * | Controls/<name>/Config/{Disabled,Slot,Restore}, Controls/<name>/Info/{Type,Motors}, Controls/<name>/Test/{Percent,RunPercent} | misc | Not plotted |
 * | Devices/<dev>/State/SupplyCurrentAmps       | f64 / device    | Motor supply (battery-side) current, amps            |
 * | Devices/<dev>/State/StatorCurrentAmps       | f64 / device    | Motor stator current, amps                           |
 * | Devices/<dev>/State/Position, Velocity, PositionRotations, VelocityRotationsPerSecond, AppliedVoltage | f64 / device | Encoder and motor readings |
 * | Devices/<dev>/Config/{NeutralMode,Inverted,StatorCurrentLimit,SupplyCurrentLimit,Disabled,Restore} | writable / control | Motor config |
 * | Devices/<dev>/Info/{Type,Kind,Id,Bus,Follower,...}, Devices/<dev>/Setup/*, Devices/<dev>/Health/* | misc | Identity, setup, IMU health |
 *
 * Setpoints and health (telemetry2):
 * - `Controls/<name>/State/{Target, Reference, ReferenceVelocity, Error, Output}` (Output is NaN when the loop runs on the motor)
 * - `Devices/<dev>/State/{TemperatureCelsius, Derate, PowerCapAmps}`, `<mech>/State/Stage`
 * - `athena/power/<motor>/{current, capAmps}` (robot level)
 *
 * Robot-level signals (outside the mechanism tree, see `ROBOT_PATHS` in data/types.ts):
 * - `athena/robot/mode`, `athena/robot/matchTime`, `athena/robot/alliance` (system; added by ArcpSystemSignals)
 * - `athena/power/battery/voltage`, `athena/power/bus/demandAmps`, `athena/power/bus/shedding` (system; same)
 * - `athena/arcp/status/{clients,bytesPerSecond,budgetBytesPerSecond,droppedFrames,throttled,throttledPatterns}` (system)
 * - `athena/locks/<lock group class>` (lock role, one boolean per lock group)
 */
import type { MechanismNode } from '../data/tree';
import { groupSignals, withinGroup } from '../data/tree';
import type { SignalDescriptor } from '../data/types';

/** Traces drawn on a control page, in display order: goal, profiled reference, measurement, error, output. */
export const PLOT_NAMES = ['Target', 'Reference', 'FeedbackPosition', 'ReferenceVelocity', 'FeedbackVelocity', 'Error', 'Output'] as const;

/** PID and feedforward leaves under `Controls/<name>/Config`, as the dashboard names them (kP, kS, ...). */
export const GAIN_NAMES: Readonly<Record<string, string>> = {
  'PID/p': 'kP',
  'PID/i': 'kI',
  'PID/d': 'kD',
  'PID/iZone': 'kIZone',
  'Feedforward/static': 'kS',
  'Feedforward/velocity': 'kV',
  'Feedforward/acceleration': 'kA',
  'Feedforward/gravity': 'kG'
};

const SHORT_GAIN_NAMES = new Set(['p', 'i', 'd', 'izone', 'kp', 'ki', 'kd', 'kizone', 'ks', 'kv', 'ka', 'kg']);

/**
 * True for a gain: either a path inside a control (`Config/PID/p`, `PID/p`, `Config/Feedforward/gravity`)
 * or a bare name (`p`, `kP`, `kG`, `iZone`).
 */
export function isGainName(name: string): boolean {
  const relative = name.startsWith('Config/') ? name.slice('Config/'.length) : name;
  if (relative in GAIN_NAMES) return true;
  return SHORT_GAIN_NAMES.has((relative.split('/').pop() ?? '').toLowerCase());
}

/** Motor current names, in preference order. Supply current is the battery-side draw used for the power split. */
export const CURRENT_NAMES = ['SupplyCurrentAmps', 'StatorCurrentAmps'] as const;
/** Device temperature (degrees C) and thermal derate (1 = full output). */
export const TEMPERATURE_NAME = 'TemperatureCelsius';
export const DERATE_NAME = 'Derate';

export type ControlSection = {
  /** Control name, e.g. `shoulder` or `fieldHeading`. */
  name: string;
  path: string;
  /** Feedback traces, in PLOT_NAMES order. */
  plot: SignalDescriptor[];
  /** PID and feedforward gains, sorted by signal name. */
  gains: SignalDescriptor[];
  constraints: SignalDescriptor[];
  other: SignalDescriptor[];
};

export type DeviceSection = {
  name: string;
  path: string;
  signals: SignalDescriptor[];
  /** Motor current signal (first of CURRENT_NAMES present), or null. */
  current: SignalDescriptor | null;
  /** Temperature signal, or null when the robot does not publish one. */
  temperature: SignalDescriptor | null;
  /** Derate signal, or null when the robot does not publish one. */
  derate: SignalDescriptor | null;
};

export type MechanismSummary = {
  /** `State/ActiveAction`: the running action, empty text when idle. There is no separate state enum. */
  state: SignalDescriptor | null;
  /** `State/Reason`. */
  reason: SignalDescriptor | null;
  /** `State/Fault`: empty text when there is no fault. */
  fault: SignalDescriptor | null;
  /** `State/ActionRunning`. */
  running: SignalDescriptor | null;
  /** `State/Stage`: the mechanism's current power stage. */
  stage: SignalDescriptor | null;
  /** Count of signals that are writable (for the page header). */
  writableCount: number;
};

/** Splits `Controls/<name>/...` signals into control sections. Signals directly under Controls go to `(root)`. */
export function controlSections(node: MechanismNode): ControlSection[] {
  const sections = new Map<string, ControlSection>();
  const sectionFor = (name: string): ControlSection => {
    let section = sections.get(name);
    if (!section) {
      section = {
        name,
        path: `${node.path}/Controls/${name}`,
        plot: [],
        gains: [],
        constraints: [],
        other: []
      };
      sections.set(name, section);
    }
    return section;
  };

  for (const signal of groupSignals(node).Controls) {
    const within = withinGroup(signal).split('/');
    if (within.length === 1) {
      sectionFor('(root)').other.push(signal);
      continue;
    }
    const [name, ...rest] = within;
    const section = sectionFor(name);
    const relative = rest.join('/');
    if (relative.startsWith('Config/Constraints/')) {
      section.constraints.push(signal);
    } else if (relative.startsWith('Config/') && relative.slice('Config/'.length) in GAIN_NAMES) {
      section.gains.push(signal);
    } else if (relative.startsWith('State/') && (PLOT_NAMES as readonly string[]).includes(rest[rest.length - 1])) {
      section.plot.push(signal);
    } else {
      section.other.push(signal);
    }
  }
  for (const section of sections.values()) {
    section.plot.sort((a, b) => PLOT_NAMES.indexOf(a.name as never) - PLOT_NAMES.indexOf(b.name as never));
    section.gains.sort((a, b) => gainOrder(a) - gainOrder(b) || a.name.localeCompare(b.name));
    section.constraints.sort((a, b) => a.name.localeCompare(b.name));
  }
  return [...sections.values()].sort((a, b) => a.name.localeCompare(b.name));
}

/** Gain order on the page: kP, kI, kD, kIZone, kS, kV, kA, kG. */
function gainOrder(signal: SignalDescriptor): number {
  // `<name>/Config/PID/p` -> `PID/p`
  return Object.keys(GAIN_NAMES).indexOf(withinGroup(signal).split('/').slice(2).join('/'));
}

/** Splits `Devices/<device>/...` signals by device and resolves the motor current, temperature and derate. */
export function deviceSections(node: MechanismNode): DeviceSection[] {
  const sections = new Map<string, DeviceSection>();
  for (const signal of groupSignals(node).Devices) {
    const [name = '(root)'] = withinGroup(signal).split('/');
    let section = sections.get(name);
    if (!section) {
      section = {
        name,
        path: `${node.path}/Devices/${name}`,
        signals: [],
        current: null,
        temperature: null,
        derate: null
      };
      sections.set(name, section);
    }
    section.signals.push(signal);
  }
  return [...sections.values()]
    .map((s) => {
      const signals = s.signals.sort((a, b) => a.path.localeCompare(b.path));
      const byName = (name: string) => signals.find((signal) => signal.name === name) ?? null;
      const current = CURRENT_NAMES.map(byName).find((signal) => signal !== null) ?? null;
      return { ...s, signals, current, temperature: byName(TEMPERATURE_NAME), derate: byName(DERATE_NAME) };
    })
    .sort((a, b) => a.name.localeCompare(b.name));
}

export function summarize(node: MechanismNode): MechanismSummary {
  const grouped = groupSignals(node);
  const find = (localPath: string) => grouped.State.find((s) => s.localPath === localPath) ?? null;
  const writableCount = node.signals.filter((s) => s.writable).length;
  return {
    state: find('State/ActiveAction'),
    reason: find('State/Reason'),
    fault: find('State/Fault'),
    running: find('State/ActionRunning'),
    stage: find('State/Stage'),
    writableCount
  };
}

/** Values that are not part of a control or device: editable setpoints and read-only readings. */
export function valueSignals(node: MechanismNode): SignalDescriptor[] {
  return groupSignals(node).Values;
}

export function actionSignals(node: MechanismNode): SignalDescriptor[] {
  return groupSignals(node).Actions.filter((s) => s.type === 'bool' || s.role === 'action');
}

/** Temperature above which the device section warns. */
export const TEMPERATURE_WARN_C = 60;

/** Derived health hint for a device signal: `warn` for temperatures, `derate` below 1, else `ok`. */
export function deviceHealth(signal: SignalDescriptor, value: number | undefined): 'ok' | 'warn' | 'derate' {
  if (value === undefined) return 'ok';
  if (signal.name === TEMPERATURE_NAME && value > TEMPERATURE_WARN_C) return 'warn';
  if (signal.name === DERATE_NAME && value < 1) return 'derate';
  return 'ok';
}
