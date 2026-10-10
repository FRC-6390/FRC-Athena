/**
 * Pure helpers for the chooser widgets (dropdown and segmented). Options come from a writable string's
 * companion `string[]` signal when one is bound, otherwise from a comma list in settings.
 */
import type { SignalDescriptor, SignalValue } from '../../data/types';

/** Comma-separated setting to a clean option list: trimmed, non-empty, first occurrence wins. */
export function parseChoices(text: unknown): string[] {
  if (typeof text !== 'string') return [];
  const seen = new Set<string>();
  for (const part of text.split(',')) {
    const item = part.trim();
    if (item !== '') seen.add(item);
  }
  return [...seen];
}

/** Options from a bound `string[]` signal when it has entries, else from the comma-list setting. */
export function resolveChoices(optionsValue: SignalValue | undefined, setting: unknown): string[] {
  if (Array.isArray(optionsValue) && optionsValue.length > 0) {
    const fromSignal = optionsValue.map((v) => String(v).trim()).filter((v) => v !== '');
    if (fromSignal.length > 0) return [...new Set(fromSignal)];
  }
  return parseChoices(setting);
}

/**
 * Options to show. The robot's current value is kept even when it is not an option (shown as extra), so the
 * user can see that the robot is somewhere the chooser does not offer.
 */
export function choiceList(options: string[], current: string | undefined): { options: string[]; extra: string | null } {
  if (current === undefined || current === '' || options.includes(current)) return { options, extra: null };
  return { options: [current, ...options], extra: current };
}

const CHOICE_NAME = /auto|mode|choice|option|select|strategy|profile|preset/i;

/** Suggestion score: writable strings first (higher when the name looks like a pick-one), options lists lower. */
export function chooserScore(signal: SignalDescriptor): number {
  if (signal.type === 'string' && signal.writable) return CHOICE_NAME.test(signal.name) ? 65 : 45;
  if (signal.type === 'string[]') return 25;
  return 0;
}
