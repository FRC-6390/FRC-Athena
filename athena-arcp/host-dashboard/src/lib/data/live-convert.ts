/**
 * Conversion between the webview's signal values and the JSON the Tauri live bridge uses
 * (`WireValue` in src-tauri/src/live.rs). `kind` names the wire type; i64 values travel as numbers.
 */
import type { SignalValue, StructValue, TypeTag } from './types';

export type WireValue =
  | { kind: 'bool'; value: boolean }
  | { kind: 'i64'; value: number }
  | { kind: 'f64'; value: number }
  | { kind: 'str'; value: string }
  | { kind: 'boolArray'; value: boolean[] }
  | { kind: 'i64Array'; value: number[] }
  | { kind: 'f64Array'; value: number[] }
  | { kind: 'strArray'; value: string[] }
  | { kind: 'struct'; value: { schemaId: number; bytes: number[] } };

/** Converts a UI value to the wire form for a signal of the given type tag. */
export function toWireValue(value: SignalValue, typeTag: TypeTag): WireValue {
  switch (typeTag) {
    case 1:
      return { kind: 'bool', value: Boolean(value) };
    case 2:
      return { kind: 'i64', value: Number(value) };
    case 3:
      return { kind: 'f64', value: Number(value) };
    case 4:
      return { kind: 'str', value: String(value) };
    case 5:
      return { kind: 'boolArray', value: value as boolean[] };
    case 6:
      return { kind: 'i64Array', value: (value as (number | bigint)[]).map(Number) };
    case 7:
      return { kind: 'f64Array', value: (value as number[]).slice() };
    case 8:
      return { kind: 'strArray', value: value as string[] };
    case 9: {
      const s = value as StructValue;
      return { kind: 'struct', value: { schemaId: s.schemaId, bytes: Array.from(s.bytes) } };
    }
    default:
      throw new Error(`unsupported type tag ${typeTag}`);
  }
}

/** Converts a wire value from the bridge back to the UI form. */
export function fromWireValue(wire: WireValue): SignalValue {
  switch (wire.kind) {
    case 'bool':
    case 'i64':
    case 'f64':
    case 'str':
    case 'boolArray':
    case 'i64Array':
    case 'f64Array':
    case 'strArray':
      return wire.value;
    case 'struct':
      return { schemaId: wire.value.schemaId, bytes: Uint8Array.from(wire.value.bytes) };
  }
}
