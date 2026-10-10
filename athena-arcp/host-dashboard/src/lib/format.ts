import type { SignalValue } from './data/types';

/** Compact display of any signal value. Numbers get up to 3 decimals; struct values show their size. */
export function formatValue(value: SignalValue | undefined): string {
  if (value === undefined) return '--';
  if (typeof value === 'number') return formatNumber(value);
  if (typeof value === 'bigint') return value.toString();
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'string') return value.length > 0 ? value : '""';
  if (Array.isArray(value)) {
    if (value.length === 0) return '[]';
    const items = value.map((v) => (typeof v === 'number' ? formatNumber(v) : String(v)));
    return `[${items.join(', ')}]`;
  }
  if (!('schemaId' in value)) return String(value);
  return `struct #${value.schemaId} (${value.bytes.length} B)`;
}

/**
 * Like {@link formatValue}, but uses the signal's declared type to decode known WPILib structs
 * (Pose2d: x, y in metres and rotation in radians, 24 bytes little-endian) into readable text.
 */
export function formatSignalValue(type: string, value: SignalValue | undefined): string {
  if (value !== undefined && typeof value === 'object' && !Array.isArray(value) && 'schemaId' in value) {
    if (type === 'struct:Pose2d' && value.bytes.length >= 24) {
      const view = new DataView(value.bytes.buffer, value.bytes.byteOffset, 24);
      const x = view.getFloat64(0, true);
      const y = view.getFloat64(8, true);
      const degrees = (view.getFloat64(16, true) * 180) / Math.PI;
      return `${x.toFixed(2)}, ${y.toFixed(2)} m · ${degrees.toFixed(1)}°`;
    }
  }
  return formatValue(value);
}

/** NaN means "no value" on Athena signals (for example a control with no target), so it shows as `--`. */
export function formatNumber(value: number): string {
  if (Number.isNaN(value)) return '--';
  if (!Number.isFinite(value)) return value > 0 ? '∞' : '-∞';
  if (Number.isInteger(value)) return Object.is(value, -0) ? '0' : value.toString();
  const abs = Math.abs(value);
  const text = abs >= 1000 ? value.toFixed(0) : abs >= 10 ? value.toFixed(1) : value.toFixed(3);
  // Tiny negatives round to "-0.000"; show them as zero.
  return /^-0(\.0+)?$/.test(text) ? text.slice(1) : text;
}

/** Seconds since the newest sample, e.g. `+12.4 s`. */
export function formatSeconds(micros: number): string {
  return `${(micros / 1_000_000).toFixed(1)} s`;
}

/**
 * Parses user-entered text for a given signal type. Returns null when the text is not valid for that type.
 * Number inputs bound with `bind:value` deliver numbers, so both forms are accepted.
 */
export function parseInput(type: string, input: string | number | null | undefined): SignalValue | null {
  // An empty number input binds as null in Svelte; treat it as empty text so validation reports it.
  const text = input === null || input === undefined ? '' : String(input);
  const trimmed = text.trim();
  switch (type) {
    case 'bool':
      if (trimmed === 'true' || trimmed === '1') return true;
      if (trimmed === 'false' || trimmed === '0') return false;
      return null;
    case 'f64':
    case 'i64': {
      if (trimmed === '') return null;
      const n = Number(trimmed);
      if (!Number.isFinite(n)) return null;
      return type === 'i64' ? Math.trunc(n) : n;
    }
    case 'string':
      return text;
    default:
      return null;
  }
}
