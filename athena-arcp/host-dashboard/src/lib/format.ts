import type { SignalValue } from './data/types';

/** Compact display of any signal value. Numbers get up to 3 decimals; struct values show their size. */
export function formatValue(value: SignalValue | undefined): string {
  if (value === undefined) return '--';
  if (typeof value === 'number') return formatNumber(value);
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'string') return value.length > 0 ? value : '""';
  if (Array.isArray(value)) {
    if (value.length === 0) return '[]';
    const items = value.map((v) => (typeof v === 'number' ? formatNumber(v) : String(v)));
    return `[${items.join(', ')}]`;
  }
  return `struct #${value.schemaId} (${value.bytes.length} B)`;
}

export function formatNumber(value: number): string {
  if (!Number.isFinite(value)) return String(value);
  if (Number.isInteger(value)) return value.toString();
  const abs = Math.abs(value);
  if (abs >= 1000) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  return value.toFixed(3);
}

/** Seconds since the newest sample, e.g. `+12.4 s`. */
export function formatSeconds(micros: number): string {
  return `${(micros / 1_000_000).toFixed(1)} s`;
}

/**
 * Parses user-entered text for a given signal type. Returns null when the text is not valid for that type.
 * Number inputs bound with `bind:value` deliver numbers, so both forms are accepted.
 */
export function parseInput(type: string, input: string | number): SignalValue | null {
  const text = typeof input === 'number' ? String(input) : input;
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
