/**
 * CSV export of replay signals over a time range (Analysis). Rows fall on every sample time of the selected
 * signals inside the range; each cell holds the value in effect at that time (sample-and-hold). Struct
 * Pose2d signals expand to `.x`, `.y`, `.theta` columns.
 */
import { decodePose2d, type ReplayModel } from './replay';
import type { SignalValue } from '../data/types';

function cell(value: SignalValue | undefined): string {
  if (value === undefined) return '';
  if (typeof value === 'boolean') return value ? '1' : '0';
  if (typeof value === 'number' || typeof value === 'bigint') return String(value);
  if (typeof value === 'string') return escapeCsv(value);
  if (Array.isArray(value)) return escapeCsv(value.join(';'));
  return escapeCsv(`struct:${value.schemaId}`);
}

/** RFC 4180 quoting, applied only when needed. */
export function escapeCsv(text: string): string {
  return /[",\n\r]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
}

export function buildCsv(
  model: ReplayModel,
  signalIds: readonly number[],
  fromMicros: number,
  toMicros: number,
  originMicros = model.startMicros
): string {
  type Column = { header: string; at: (micros: number) => string };
  const columns: Column[] = [];
  const times = new Set<number>();

  for (const id of signalIds) {
    const descriptor = model.descriptor(id);
    if (!descriptor) continue;
    const track = model.tracks.get(id);
    if (!track) continue;
    const inRange = track.t.filter((t) => t >= fromMicros && t <= toMicros);
    for (const t of inRange) times.add(t);
    if (descriptor.type === 'struct:Pose2d') {
      for (const part of ['x', 'y', 'theta'] as const) {
        columns.push({
          header: `${descriptor.path}.${part}`,
          at: (micros) => {
            const value = model.valueAt(id, micros);
            if (typeof value !== 'object' || value === null || !('bytes' in value)) return '';
            const decoded = decodePose2d(value.bytes);
            return decoded ? String(decoded[part]) : '';
          }
        });
      }
    } else {
      const unit = descriptor.unit ? ` [${descriptor.unit}]` : '';
      columns.push({ header: `${descriptor.path}${unit}`, at: (micros) => cell(model.valueAt(id, micros)) });
    }
  }

  // The range start always gets a row so the export shows the value in effect when the range opens.
  if (columns.length > 0) times.add(fromMicros);
  const sorted = [...times].filter((t) => t >= fromMicros && t <= toMicros).sort((a, b) => a - b);
  const lines = [['time_s', ...columns.map((c) => escapeCsv(c.header))].join(',')];
  for (const micros of sorted) {
    const row = [((micros - originMicros) / 1_000_000).toFixed(6), ...columns.map((c) => c.at(micros))];
    lines.push(row.join(','));
  }
  return lines.join('\n') + '\n';
}
