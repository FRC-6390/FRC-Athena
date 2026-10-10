import type { Component } from 'svelte';
import type { RobotCommands } from '../data/commands';
import type { ValueStore } from '../data/store';
import type { SignalDescriptor, SignalType } from '../data/types';
import type { SignalWidgetKind } from './kinds';

/** Classic signal widgets plus modules that exist only on the new contract. */
export type WidgetModuleId = SignalWidgetKind | 'plot';

/**
 * Typed widget contract (F4). Each widget module declares what it accepts, how it is accessed, and
 * optionally a component. The registry (`registry.ts`) is the only place that decides which modules a
 * signal can be shown with; choosing or dropping a signal offers only compatible modules.
 *
 * Type patterns:
 *   exact type   'f64', 'bool', 'string', 'f64[]', 'struct:Pose2d'
 *   'numeric'    i64 | f64
 *   'struct:*'   any struct
 *   '*'          anything
 */
export type TypePattern = SignalType | 'numeric' | 'struct:*' | '*';

export type WidgetAccess = 'read' | 'write' | 'any';

/** Props given to modules that render through the new contract. */
export type WidgetProps<C> = {
  signal: SignalDescriptor;
  config: C;
  store: ValueStore;
  commands: RobotCommands;
};

/** The signal shape every module matches against. Both `SignalDescriptor` and classic rows adapt to it. */
export type CompatSignal = {
  path: string;
  /** Wire type spelling, e.g. `f64`, `bool[]`, `struct:Pose2d`. */
  type: string;
  /** Role name (`action`, `state`, `tunable`, `metric`, `value`, ...). */
  role: string;
  writable: boolean;
};

export type WidgetModule<C = unknown> = {
  /** Stable id; matches the saved layout `kind` for classic widgets. */
  id: WidgetModuleId;
  title: string;
  summary: string;
  accepts: readonly TypePattern[];
  /** `write` needs a writable signal, `read` needs a read-only one, `any` takes both. */
  access: WidgetAccess;
  /** When set, only signals with these roles qualify. */
  roles?: readonly string[];
  /** Signals with these roles qualify whatever their type (action signals show as state/text). */
  acceptsRoles?: readonly string[];
  /** Signals with these roles never qualify (keeps action signals out of metric widgets). */
  excludeRoles?: readonly string[];
  /** Restricts the module to signals whose path matches (hardware widgets). */
  pathHint?: (path: string) => boolean;
  /**
   * Offered in the classic palette when no signal is selected. Modules that need a bound signal to make
   * sense (controller, action, tunable, camera overlay) set this to false.
   */
  offeredWithoutSignal?: boolean;
  /**
   * `legacy` modules are rendered by the classic canvas through their component in
   * `components/widget-registry.ts`. New-contract modules (plot) are not offered in the classic palette.
   */
  legacy: boolean;
  /** Typed config reader for new-contract modules. */
  readConfig?: (raw: Record<string, unknown> | undefined) => C;
};

/**
 * Svelte components for new-contract modules are bound outside `lib/` (see
 * `components/widget-registry.ts`) so the metadata registry stays importable from plain TypeScript tests.
 */
export type WidgetComponent<C = unknown> = Component<WidgetProps<C>>;

const NUMERIC: readonly string[] = ['i64', 'f64'];

/** True when a single type pattern matches a concrete signal type. */
export function typeMatches(pattern: TypePattern, type: string): boolean {
  if (pattern === '*') return true;
  if (pattern === 'numeric') return NUMERIC.includes(type);
  if (pattern === 'struct:*') return type.startsWith('struct:');
  return pattern === type;
}

export function accessMatches(access: WidgetAccess, writable: boolean): boolean {
  return access === 'any' || (access === 'write') === writable;
}
