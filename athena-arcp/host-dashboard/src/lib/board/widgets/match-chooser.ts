/** Auto chooser options and selection state. Pure. */

/** Splits a `|` separated option list, trimming blanks and duplicates. */
export function parseOptions(text: string): string[] {
  const seen = new Set<string>();
  const out: string[] = [];
  for (const raw of text.split('|')) {
    const item = raw.trim();
    if (item === '' || seen.has(item)) continue;
    seen.add(item);
    out.push(item);
  }
  return out;
}

/** Options from the robot's list when it publishes one, else the typed list. */
export function chooserOptions(fromSignal: readonly string[] | undefined, fromSetting: string): string[] {
  if (fromSignal && fromSignal.length > 0) return [...new Set(fromSignal)];
  return parseOptions(fromSetting);
}

export type Selection = {
  /** Option that matches the robot's current value, or null. */
  selected: string | null;
  /** The robot reports a value that is not in the list. */
  unknownValue: string | null;
};

export function selectionState(options: readonly string[], current: unknown): Selection {
  const value = typeof current === 'string' ? current : null;
  if (value === null || value === '') return { selected: null, unknownValue: null };
  return options.includes(value) ? { selected: value, unknownValue: null } : { selected: null, unknownValue: value };
}
