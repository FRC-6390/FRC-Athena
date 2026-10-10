import type { Session } from '../data/session';
import type { SignalDescriptor } from '../data/types';
import type { WidgetContext, WidgetDef, WidgetInstance } from './types';
import { humanLabel } from './widgets/controls-values';

/** Resolves a widget's bindings (stored as paths) against the live dictionary. */
export function resolveBindings(session: Session, instance: WidgetInstance): Record<string, SignalDescriptor[]> {
  const out: Record<string, SignalDescriptor[]> = {};
  for (const [slot, paths] of Object.entries(instance.bindings)) {
    out[slot] = paths
      .map((path) => session.store.descriptorByPath(path))
      .filter((d): d is SignalDescriptor => d !== undefined);
  }
  return out;
}

export function createContext(options: {
  session: Session;
  live: { readonly current: number };
  instance: WidgetInstance;
  def: WidgetDef;
  bindings: Record<string, SignalDescriptor[]>;
  size: { width: number; height: number };
  editing: boolean;
}): WidgetContext {
  const { session, live, instance, def, bindings, size, editing } = options;
  const store = session.store;
  const defaults = new Map(def.settings.map((f) => [f.key, f.default]));
  return {
    widgetId: instance.id,
    session,
    live,
    bindings,
    settings: instance.settings,
    size,
    editing,
    latest: (signal) => (signal ? store.latestSample(signal.signalId)?.value : undefined),
    samples: (signal) => store.samples(signal.signalId),
    reply: (signal) => store.reply(signal.signalId),
    locked: (signal) => store.lockStateOf(signal.signalId) === 'locked',
    write: (signal, value) => session.commands.write(signal.signalId, value),
    run: (signal) => session.commands.run(signal.signalId),
    cancel: (signal) => session.commands.cancel(signal.signalId),
    setting: <T>(key: string) => (key in instance.settings ? instance.settings[key] : defaults.get(key)) as T
  };
}

/** Display title: explicit title, else the primary binding's name, else the widget type name. */
export function widgetTitle(instance: WidgetInstance, def: WidgetDef, bindings: Record<string, SignalDescriptor[]>): string {
  if (instance.title && instance.title.trim() !== '') return instance.title;
  const first = def.slots.map((s) => bindings[s.key]?.[0]).find((d) => d !== undefined);
  if (!first || def.titleFrom === 'fixed') return def.name;
  // Derived names are code identifiers (`followerElevator`, `chassisImu`); titles read as words.
  const mechanism = humanLabel(first.mechanismPath.split('/').pop() ?? '');
  const group = (name: string) => {
    const parts = first.localPath.split('/');
    return parts[0] === name && parts[1] ? humanLabel(parts[1]) : null;
  };
  const signalName = humanLabel(first.name);
  switch (def.titleFrom) {
    case 'mechanism':
      return mechanism || signalName;
    case 'device':
      return [mechanism, group('Devices')].filter(Boolean).join(' · ') || signalName;
    case 'control':
      return [mechanism, group('Controls')].filter(Boolean).join(' · ') || signalName;
    default:
      return mechanism && mechanism !== signalName ? `${mechanism} · ${signalName}` : signalName;
  }
}
