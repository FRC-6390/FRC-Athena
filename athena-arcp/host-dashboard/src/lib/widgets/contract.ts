import type { Component } from 'svelte';
import type { RobotCommands } from '../data/commands';
import type { ValueStore } from '../data/store';
import type { SignalDescriptor, SignalRole, SignalType } from '../data/types';
import type { WidgetKind } from '../dashboard';

/** Legacy widget kinds plus modules that exist only on the new contract. */
export type WidgetModuleId = WidgetKind | 'plot';

/**
 * Typed widget contract (F4). Each widget module declares what it accepts, how it is accessed, and
 * optionally a component. The registry (`registry.ts`) lists modules; choosing or dropping a signal
 * offers only compatible modules.
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

export type WidgetModule<C = unknown> = {
  /** Stable id; matches the saved layout `kind` for legacy widgets. */
  id: WidgetModuleId;
  title: string;
  summary: string;
  accepts: readonly TypePattern[];
  /** `write` needs a writable signal, `read` needs a read-only one, `any` takes both. */
  access: WidgetAccess;
  /** When set, only signals with these roles qualify. */
  roles?: readonly SignalRole[];
  /** Signals with these roles never qualify (keeps action signals out of metric widgets). */
  excludeRoles?: readonly SignalRole[];
  /** Restricts the module to signals whose path matches (hardware widgets). */
  pathHint?: (path: string) => boolean;
  /**
   * `legacy` modules are still rendered by the classic WidgetCanvas; they carry metadata only until that
   * canvas is moved onto the contract.
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

const NUMERIC: SignalType[] = ['i64', 'f64'];

/** True when a single type pattern matches a concrete signal type. */
export function typeMatches(pattern: TypePattern, type: SignalType): boolean {
  if (pattern === '*') return true;
  if (pattern === 'numeric') return NUMERIC.includes(type);
  if (pattern === 'struct:*') return type.startsWith('struct:');
  return pattern === type;
}

export function accessMatches(access: WidgetAccess, writable: boolean): boolean {
  return access === 'any' || (access === 'write') === writable;
}
