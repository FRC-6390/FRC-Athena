import type { SignalDescriptor } from '../../../lib/data/types';

/** Numeric signal families accepted by number slots. */
export const NUMBER = ['f64', 'i64'];

export const ALLIANCE_TRUE_OPTIONS = [
  { value: 'red', label: 'Red' },
  { value: 'blue', label: 'Blue' }
];

export const isNumeric = (s: SignalDescriptor) => s.type === 'f64' || s.type === 'i64';
export const isString = (s: SignalDescriptor) => s.type === 'string';
export const isBoolean = (s: SignalDescriptor) => s.type === 'bool';

/** Matches on the signal's path, so suggestions favour names such as `battery_voltage` or `stream_url`. */
export const nameMatches = (s: SignalDescriptor, pattern: RegExp) => pattern.test(s.path);
