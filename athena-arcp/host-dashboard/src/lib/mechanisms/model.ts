import type { MechanismNode } from '../data/tree';
import { groupSignals, withinGroup } from '../data/tree';
import type { SignalDescriptor } from '../data/types';

/** Control signal names drawn as traces, in display order (telemetry-and-simulation.md, Controls). */
export const PLOT_NAMES = ['target', 'reference', 'measured'] as const;

/** Gain names such as kP, kI, kD, kS, kV, kA, kG, kF. */
export function isGainName(name: string): boolean {
  return /^k[A-Z]{1,2}$/.test(name);
}

export type ControlSection = {
  /** Control name, e.g. `position`. */
  name: string;
  path: string;
  /** target / reference / measured, in PLOT_NAMES order. */
  plot: SignalDescriptor[];
  gains: SignalDescriptor[];
  constraints: SignalDescriptor[];
  other: SignalDescriptor[];
};

export type DeviceSection = {
  name: string;
  path: string;
  signals: SignalDescriptor[];
};

export type MechanismSummary = {
  state: SignalDescriptor | null;
  reason: SignalDescriptor | null;
  stage: SignalDescriptor | null;
  /** Count of signals that are writable (for the page header). */
  writableCount: number;
};

/** Splits `Controls/<name>/rest` signals into control sections. Signals directly under Controls go to `other`. */
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
    const leaf = rest[rest.length - 1];
    if (rest.includes('Constraints') || /^(min|max)/.test(leaf)) {
      section.constraints.push(signal);
    } else if (rest.length === 1 && isGainName(leaf)) {
      section.gains.push(signal);
    } else if (rest.length === 1 && (PLOT_NAMES as readonly string[]).includes(leaf)) {
      section.plot.push(signal);
    } else {
      section.other.push(signal);
    }
  }
  for (const section of sections.values()) {
    section.plot.sort((a, b) => PLOT_NAMES.indexOf(a.name as never) - PLOT_NAMES.indexOf(b.name as never));
    section.gains.sort((a, b) => a.name.localeCompare(b.name));
    section.constraints.sort((a, b) => a.name.localeCompare(b.name));
  }
  return [...sections.values()].sort((a, b) => a.name.localeCompare(b.name));
}

/** Splits `Devices/<device>/...` signals by device. */
export function deviceSections(node: MechanismNode): DeviceSection[] {
  const sections = new Map<string, DeviceSection>();
  for (const signal of groupSignals(node).Devices) {
    const [name = '(root)'] = withinGroup(signal).split('/');
    let section = sections.get(name);
    if (!section) {
      section = { name, path: `${node.path}/Devices/${name}`, signals: [] };
      sections.set(name, section);
    }
    section.signals.push(signal);
  }
  return [...sections.values()]
    .map((s) => ({ ...s, signals: s.signals.sort((a, b) => a.name.localeCompare(b.name)) }))
    .sort((a, b) => a.name.localeCompare(b.name));
}

export function summarize(node: MechanismNode): MechanismSummary {
  const grouped = groupSignals(node);
  const find = (name: string) => grouped.State.find((s) => s.name === name) ?? null;
  const writableCount = node.signals.filter((s) => s.writable).length;
  return { state: find('state'), reason: find('reason'), stage: find('stage'), writableCount };
}

/** Values that are not part of a control or device: editable setpoints and read-only readings. */
export function valueSignals(node: MechanismNode): SignalDescriptor[] {
  return groupSignals(node).Values;
}

export function actionSignals(node: MechanismNode): SignalDescriptor[] {
  return groupSignals(node).Actions.filter((s) => s.type === 'bool' || s.role === 'action');
}

/** Temperature above which the device section warns (matches the mock derate curve start). */
export const TEMPERATURE_WARN_C = 60;

/** Derived health hint for a device signal: `warn` for temperatures, `derate` below 1, else `ok`. */
export function deviceHealth(signal: SignalDescriptor, value: number | undefined): 'ok' | 'warn' | 'derate' {
  if (value === undefined) return 'ok';
  if (signal.name === 'temperature' && value > TEMPERATURE_WARN_C) return 'warn';
  if (signal.name === 'derate' && value < 1) return 'derate';
  return 'ok';
}
