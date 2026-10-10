/**
 * The simulated robot's real dictionary (example-projects/mechanism-examples, headless sim), trimmed to paths,
 * types, roles and writability. Tests build descriptors from it so helpers are proven against real names.
 * Regenerate from a running bridge when the robot's signal names change.
 */
import { describePath } from '../path';
import { TYPE_TAG, type SignalDescriptor, type SignalRole, type SignalType } from '../types';
import entries from './real-dictionary.json';

type FixtureEntry = { path: string; type: SignalType; role: SignalRole; writable: boolean };

/** Every fixture signal as a descriptor. Signal ids are the fixture order, starting at 1. */
export function realDictionary(): SignalDescriptor[] {
  return (entries as FixtureEntry[]).map((entry, index) => ({
    signalId: index + 1,
    ...describePath(entry.path),
    type: entry.type,
    typeTag: typeTagOf(entry.type),
    unit: '',
    role: entry.role,
    writable: entry.writable,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0
  }));
}

/** Descriptor for one signal that is not in the fixture (for example a robot signal added after the capture). */
export function descriptorFor(
  signalId: number,
  path: string,
  type: SignalType = 'f64',
  role: SignalRole = 'value'
): SignalDescriptor {
  return {
    signalId,
    ...describePath(path),
    type,
    typeTag: typeTagOf(type),
    unit: '',
    role,
    writable: false,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0
  };
}

function typeTagOf(type: SignalType): SignalDescriptor['typeTag'] {
  if (type === 'bool') return TYPE_TAG.bool;
  if (type === 'string') return TYPE_TAG.string;
  if (type.startsWith('struct:')) return TYPE_TAG.struct;
  return TYPE_TAG.f64;
}
