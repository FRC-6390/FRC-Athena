import type { MechanismGroup } from './path';
import type { SignalDescriptor } from './types';

/** One mechanism in the generated tree. Child mechanisms stay nested under the declaring mechanism. */
export type MechanismNode = {
  path: string;
  name: string;
  depth: number;
  children: MechanismNode[];
  /** Signals whose mechanismPath is exactly this node's path. */
  signals: SignalDescriptor[];
};

export type GroupedSignals = Record<MechanismGroup, SignalDescriptor[]>;

/** Builds the mechanism tree from dictionary entries. Intermediate parents are created implicitly. */
export function buildMechanismTree(signals: readonly SignalDescriptor[]): MechanismNode[] {
  const nodes = new Map<string, MechanismNode>();
  const roots: MechanismNode[] = [];

  const ensure = (path: string): MechanismNode => {
    const existing = nodes.get(path);
    if (existing) return existing;
    const segments = path.split('/');
    const parentPath = segments.slice(0, -1).join('/');
    const node: MechanismNode = {
      path,
      name: segments[segments.length - 1] ?? path,
      depth: segments.length - 1,
      children: [],
      signals: []
    };
    nodes.set(path, node);
    if (parentPath && parentPath !== '') {
      ensure(parentPath).children.push(node);
    } else {
      roots.push(node);
    }
    return node;
  };

  for (const signal of signals) {
    // Lock-state and system signals describe the robot, not a mechanism; they stay in the dictionary.
    if (signal.mechanismPath === '' || signal.role === 'lock' || signal.role === 'system') continue;
    ensure(signal.mechanismPath).signals.push(signal);
  }
  for (const node of nodes.values()) {
    node.children.sort((a, b) => a.path.localeCompare(b.path));
    node.signals.sort((a, b) => a.path.localeCompare(b.path));
  }
  return roots.sort((a, b) => a.path.localeCompare(b.path));
}

/** Depth-first list of every mechanism node. */
export function flattenMechanisms(roots: readonly MechanismNode[]): MechanismNode[] {
  const out: MechanismNode[] = [];
  const visit = (node: MechanismNode) => {
    out.push(node);
    node.children.forEach(visit);
  };
  roots.forEach(visit);
  return out;
}

/** Splits a mechanism's own signals by group (State, Values, Actions, ...). */
export function groupSignals(node: MechanismNode): GroupedSignals {
  const grouped: GroupedSignals = {
    State: [],
    Values: [],
    Actions: [],
    Devices: [],
    Controls: [],
    Hooks: [],
    Trace: []
  };
  for (const signal of node.signals) {
    const group = signal.localPath.split('/')[0] as MechanismGroup;
    if (group in grouped) grouped[group].push(signal);
  }
  return grouped;
}

/** Local path inside a group, e.g. `Controls/position/kP` -> `position/kP`. */
export function withinGroup(signal: SignalDescriptor): string {
  return signal.localPath.split('/').slice(1).join('/');
}
