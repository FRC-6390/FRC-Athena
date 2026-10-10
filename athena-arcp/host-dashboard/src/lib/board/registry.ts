import type { SignalDescriptor } from '../data/types';
import type { BindingSlot, SettingField, SignalMatcher, WidgetDef, WidgetInstance } from './types';

const definitions = new Map<string, WidgetDef>();

export function registerWidget(def: WidgetDef): void {
  if (definitions.has(def.type)) throw new Error(`Widget type "${def.type}" is already registered`);
  definitions.set(def.type, def);
}

export function widgetDef(type: string): WidgetDef | undefined {
  return definitions.get(type);
}

export function allWidgets(): WidgetDef[] {
  return [...definitions.values()];
}

/** Test hook. */
export function clearRegistry(): void {
  definitions.clear();
}

export function matches(matcher: SignalMatcher, signal: SignalDescriptor): boolean {
  if (matcher.writable && !signal.writable) return false;
  if (matcher.action && signal.role !== 'action' && !signal.localPath.startsWith('Actions/')) return false;
  if (matcher.types && matcher.types.length > 0) {
    const ok = matcher.types.some((t) => (t.endsWith('*') ? signal.type.startsWith(t.slice(0, -1)) : signal.type === t));
    if (!ok) return false;
  }
  return matcher.test ? matcher.test(signal) : true;
}

/** The slot a dropped signal should fill: the first required slot it matches, else any matching slot. */
export function primarySlot(def: WidgetDef, signal: SignalDescriptor): BindingSlot | undefined {
  return def.slots.find((s) => s.required && matches(s.matcher, signal)) ?? def.slots.find((s) => matches(s.matcher, signal));
}

/** Widgets that can show `signal`, best first. */
export function suggestWidgets(signal: SignalDescriptor, defs: WidgetDef[] = allWidgets()): WidgetDef[] {
  return defs
    .filter((def) => primarySlot(def, signal) !== undefined)
    .map((def) => ({ def, score: def.score?.(signal) ?? 0 }))
    .sort((a, b) => b.score - a.score || a.def.name.localeCompare(b.def.name))
    .map((entry) => entry.def);
}

export function defaultSettings(fields: SettingField[]): Record<string, unknown> {
  return Object.fromEntries(fields.map((f) => [f.key, f.default]));
}

let counter = 0;
export function newId(prefix = 'w'): string {
  counter += 1;
  return `${prefix}-${Date.now().toString(36)}-${counter.toString(36)}`;
}

/** A new widget instance at a grid position, with `signal` bound to its primary slot. */
export function createWidget(def: WidgetDef, at: { x: number; y: number }, signal?: SignalDescriptor): WidgetInstance {
  const bindings: Record<string, string[]> = {};
  if (signal) {
    const slot = primarySlot(def, signal);
    if (slot) bindings[slot.key] = [signal.path];
  }
  return {
    id: newId(),
    type: def.type,
    x: at.x,
    y: at.y,
    w: def.defaultSize.w,
    h: def.defaultSize.h,
    bindings,
    settings: defaultSettings(def.settings)
  };
}
