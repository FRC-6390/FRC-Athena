import { describe, expect, test } from 'bun:test';
import { fromWireValue, toWireValue } from './live-convert';

describe('live value conversion', () => {
  test('scalars and arrays keep their wire kind', () => {
    expect(toWireValue(true, 1)).toEqual({ kind: 'bool', value: true });
    expect(toWireValue(12n, 2)).toEqual({ kind: 'i64', value: 12 });
    expect(toWireValue(1.5, 3)).toEqual({ kind: 'f64', value: 1.5 });
    expect(toWireValue('hi', 4)).toEqual({ kind: 'str', value: 'hi' });
    expect(toWireValue([1, 2], 7)).toEqual({ kind: 'f64Array', value: [1, 2] });
    expect(toWireValue(['a'], 8)).toEqual({ kind: 'strArray', value: ['a'] });
  });

  test('structs carry the schema id and bytes', () => {
    const wire = toWireValue({ schemaId: 3, bytes: new Uint8Array([1, 2]) }, 9);
    expect(wire).toEqual({ kind: 'struct', value: { schemaId: 3, bytes: [1, 2] } });
    const back = fromWireValue(wire);
    expect(back).toEqual({ schemaId: 3, bytes: new Uint8Array([1, 2]) });
  });

  test('unknown type tags are rejected', () => {
    expect(() => toWireValue(1, 99 as never)).toThrow();
  });
});
