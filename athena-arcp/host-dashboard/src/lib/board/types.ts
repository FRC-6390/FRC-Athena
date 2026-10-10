import type { Component } from 'svelte';
import type { Session } from '../data/session';
import type { CommandReply } from '../data/commands';
import type { Sample, SignalDescriptor, SignalValue } from '../data/types';

/** A saved board: a named grid of widgets. Positions are in grid cells. */
export type BoardDoc = {
  id: string;
  name: string;
  /** Number of grid columns (the board scales to the window width). */
  columns: number;
  widgets: WidgetInstance[];
  /** Locked boards open in drive mode and need an explicit unlock to edit (driver stations). */
  locked?: boolean;
};

export type WidgetInstance = {
  id: string;
  /** Registry type, e.g. `value`, `graph`. */
  type: string;
  x: number;
  y: number;
  w: number;
  h: number;
  /** Optional title override; widgets fall back to their binding's name. */
  title?: string;
  /** Slot key -> signal paths. Paths (not ids) so boards survive reconnects and robot restarts. */
  bindings: Record<string, string[]>;
  settings: Record<string, unknown>;
};

/** Which signals a binding slot accepts. */
export type SignalMatcher = {
  /** Accepted signal types; prefixes ending in `*` match families, e.g. `struct:*`, `f64[]`. Empty = any. */
  types?: string[];
  /** Only writable signals (inputs, toggles, sliders). */
  writable?: boolean;
  /** Only action signals. */
  action?: boolean;
  /** Extra predicate for special widgets (e.g. a field accepts only `struct:Pose2d`). */
  test?: (signal: SignalDescriptor) => boolean;
};

export type BindingSlot = {
  key: string;
  label: string;
  matcher: SignalMatcher;
  /** Several signals (e.g. graph series). */
  multiple?: boolean;
  required?: boolean;
};

export type SettingField =
  | { key: string; label: string; kind: 'number'; default: number; min?: number; max?: number; step?: number; unit?: string }
  | { key: string; label: string; kind: 'text'; default: string; placeholder?: string }
  | { key: string; label: string; kind: 'textarea'; default: string; placeholder?: string; rows?: number }
  | { key: string; label: string; kind: 'bool'; default: boolean }
  | { key: string; label: string; kind: 'enum'; default: string; options: { value: string; label: string }[] }
  | { key: string; label: string; kind: 'color'; default: string };

export type WidgetCategory = 'Values' | 'Controls' | 'Charts' | 'Robot' | 'Match' | 'Layout';

/** Everything a widget component receives. Helpers keep widgets free of store plumbing. */
export type WidgetContext = {
  /** Stable id of this widget instance (for per-widget local state such as checklist ticks). */
  widgetId: string;
  session: Session;
  live: { readonly current: number };
  /** Slot key -> resolved descriptors (missing signals are left out). */
  bindings: Record<string, SignalDescriptor[]>;
  settings: Record<string, unknown>;
  /** Pixel size of the widget body, for widgets that scale their content. */
  size: { width: number; height: number };
  editing: boolean;
  latest(signal: SignalDescriptor | undefined): SignalValue | undefined;
  samples(signal: SignalDescriptor): Sample[];
  reply(signal: SignalDescriptor): CommandReply | undefined;
  locked(signal: SignalDescriptor): boolean;
  write(signal: SignalDescriptor, value: SignalValue): Promise<CommandReply>;
  run(signal: SignalDescriptor): Promise<CommandReply>;
  cancel(signal: SignalDescriptor): Promise<CommandReply>;
  /** Setting with its default applied. */
  setting<T>(key: string): T;
};

export type WidgetDef = {
  type: string;
  name: string;
  category: WidgetCategory;
  description: string;
  slots: BindingSlot[];
  settings: SettingField[];
  defaultSize: { w: number; h: number };
  minSize?: { w: number; h: number };
  /**
   * Where the automatic title comes from: the bound signal's name (default), or the mechanism, device or
   * control it belongs to (for cards that show a whole group).
   */
  titleFrom?: 'signal' | 'mechanism' | 'device' | 'control' | 'fixed';
  /** Suggestion priority when a signal is dropped (higher first). */
  score?: (signal: SignalDescriptor) => number;
  component: Component<{ ctx: WidgetContext }>;
};
