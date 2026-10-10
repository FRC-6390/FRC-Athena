/**
 * Adapter between the v2 data layer (Session / ValueStore) and the classic canvas.
 *
 * The classic canvas (WidgetCanvas, widgets, inspector, config editor) consumes `SignalRow[]` and two
 * commands. This module produces both from a v2 session without touching the data layer itself.
 */
import type { SignalRow } from './arcp';
import type { CommandReply, SessionCommands } from './data/commands';
import type { ValueStore } from './data/store';
import type { Sample, SignalDescriptor, SignalValue } from './data/types';
import { parseInput } from './format';

/** Value shown for a signal that has not reported a sample yet (the classic convention). */
export const MISSING_VALUE = '-';

/** The slice of a ValueStore the adapter reads. Keeps tests free of a live store. */
export type CanvasStoreView = Pick<ValueStore, 'allSignals' | 'latestSample' | 'descriptor'>;

/**
 * Commands the classic canvas calls. `raw` is the text the user typed. Each resolves to an error message
 * (rejected locally, or refused by the robot) or null when the command was accepted.
 */
export type CanvasCommands = {
  setValue(signalId: number, raw: string): Promise<string | null>;
  triggerAction(signalId: number): Promise<string | null>;
};

/** Classic `access` strings: `observe` for readable values, `write` for tunables, `invoke` for actions. */
export function classicAccess(signal: SignalDescriptor): 'observe' | 'write' | 'invoke' {
  if (signal.role === 'action') return 'invoke';
  if (signal.writable) return 'write';
  return 'observe';
}

/** Renders a v2 value as the string classic widgets parse. Arrays are JSON, bools are true/false. */
export function classicValueText(value: SignalValue | undefined): string {
  if (value === undefined) return MISSING_VALUE;
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'number' || typeof value === 'bigint') return String(value);
  if (typeof value === 'string') return value;
  if (Array.isArray(value)) {
    return JSON.stringify(value.map((item) => (typeof item === 'bigint' ? Number(item) : item)));
  }
  // Struct values: widgets cannot decode them; show the size like the v2 table does.
  return `struct #${value.schemaId} (${value.bytes.length} B)`;
}

/** Maps one v2 descriptor and its latest sample to a classic row. */
export function toClassicRow(signal: SignalDescriptor, sample: Sample | undefined): SignalRow {
  const access = classicAccess(signal);
  return {
    signal_id: signal.signalId,
    signal_type: signal.type,
    kind: access === 'invoke' ? 'command' : 'value',
    access,
    // Booleans change on events, numerics are sampled periodically; durability follows persistence.
    policy: signal.type === 'bool' ? 'on_change' : 'periodic',
    durability: signal.persisted ? 'persistent' : 'volatile',
    path: signal.path,
    value: classicValueText(sample?.value)
  };
}

/** Classic rows for every dictionary entry, sorted by path (the store already sorts). */
export function toClassicRows(store: CanvasStoreView): SignalRow[] {
  return store.allSignals().map((signal) => toClassicRow(signal, store.latestSample(signal.signalId)));
}

/**
 * Parses text typed into a classic widget for the signal's wire type. Arrays take a JSON array; scalars go
 * through the shared parser. Returns null when the text is not valid for the type.
 */
export function parseClassicValue(type: string, raw: string): SignalValue | null {
  if (type.endsWith('[]')) {
    try {
      const parsed = JSON.parse(raw.trim()) as unknown;
      if (!Array.isArray(parsed)) return null;
      const elementType = type.slice(0, -2);
      const items = parsed.map((item) => parseInput(elementType, item as string | number));
      if (items.some((item) => item === null)) return null;
      return items as SignalValue;
    } catch {
      return null;
    }
  }
  return parseInput(type, raw);
}

/** Error text for a robot reply that did not accept the command, or null when it did. */
export function replyError(reply: CommandReply): string | null {
  return reply.accepted ? null : reply.reason || 'rejected by robot';
}

/**
 * Commands for the classic canvas on top of a session. `setValue` parses the text for the signal's type
 * and writes it; text that does not parse is reported and nothing is sent. Writes are not otherwise
 * pre-validated (the robot decides).
 */
export function classicCommands(
  store: Pick<CanvasStoreView, 'descriptor'>,
  commands: Pick<SessionCommands, 'write' | 'run'>
): CanvasCommands {
  return {
    async setValue(signalId, raw) {
      const descriptor = store.descriptor(signalId);
      if (!descriptor) return `unknown signal ${signalId}`;
      const value = parseClassicValue(descriptor.type, raw);
      if (value === null) return `"${raw.trim()}" is not a valid ${descriptor.type} value`;
      return replyError(await commands.write(signalId, value));
    },
    async triggerAction(signalId) {
      return replyError(await commands.run(signalId));
    }
  };
}

/** Signals on the robot's NT4 compatibility tree (shown in the explorer's NT list, not in the main list). */
export function isNtCompatPath(path: string): boolean {
  return path.startsWith('Athena/NT4/') || path.startsWith('NT4/');
}

/** Publish-only plumbing signals hidden from every list. */
export function isIgnoredPublishSignal(signal: SignalRow): boolean {
  const token = signal.path.toLowerCase().replace(/[^a-z0-9]+/g, '');
  return token.includes('publish');
}
