import { SignalRow } from '../arcp';
import { WidgetConfigRecord } from '../dashboard';
import { isObject, toBoolean, toNumber, toString } from './common';

export const HTML_INPUT_TYPES = [
  'text',
  'search',
  'password',
  'email',
  'url',
  'tel',
  'number',
  'range',
  'date',
  'time',
  'datetime-local',
  'month',
  'week',
  'color'
] as const;

export type DropdownOption = {
  label: string;
  value: string;
};

export type DropdownWidgetCommit = 'auto' | 'button';

export type DropdownWidgetConfig = {
  options: DropdownOption[];
  commit: DropdownWidgetCommit;
  buttonLabel: string;
};

export type ToggleWidgetStyle = 'switch' | 'button';

export type ToggleWidgetConfig = {
  trueLabel: string;
  falseLabel: string;
  style: ToggleWidgetStyle;
};

export type ChoiceWidgetDirection = 'vertical' | 'horizontal';

export type ChoiceWidgetCommit = 'auto' | 'button';

export type ChoiceWidgetConfig = {
  options: DropdownOption[];
  direction: ChoiceWidgetDirection;
  commit: ChoiceWidgetCommit;
  buttonLabel: string;
};

export type HtmlInputType = (typeof HTML_INPUT_TYPES)[number];

export type InputWidgetCommit = 'auto' | 'enter' | 'blur' | 'button';

export type InputWidgetConfig = {
  inputType: HtmlInputType;
  placeholder: string;
  commit: InputWidgetCommit;
  buttonLabel: string;
  min: string;
  max: string;
  step: string;
  pattern: string;
  autocomplete: string;
  spellcheck: boolean;
};

export type TextAreaWidgetCommit = 'auto' | 'ctrl-enter' | 'blur' | 'button';

export type TextAreaWidgetConfig = {
  placeholder: string;
  rows: number;
  maxLength: number | null;
  commit: TextAreaWidgetCommit;
  buttonLabel: string;
  spellcheck: boolean;
};

export function isHtmlInputType(value: string): value is HtmlInputType {
  return (HTML_INPUT_TYPES as readonly string[]).includes(value);
}

export function defaultInputTypeForSignal(signal: SignalRow): HtmlInputType {
  if (signal.signal_type === 'f64' || signal.signal_type === 'i64') return 'number';
  return 'text';
}

export function defaultInputCommitForType(inputType: HtmlInputType): InputWidgetCommit {
  if (inputType === 'range' || inputType === 'color' || inputType === 'date' || inputType === 'time') {
    return 'auto';
  }
  return 'enter';
}

export function readInputConfig(
  raw: WidgetConfigRecord | undefined,
  signal: SignalRow
): InputWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const fallbackType = defaultInputTypeForSignal(signal);
  const inputTypeRaw = toString(base.inputType, fallbackType).toLowerCase();
  const inputType = isHtmlInputType(inputTypeRaw) ? inputTypeRaw : fallbackType;

  const commitRaw = toString(base.commit, defaultInputCommitForType(inputType)).toLowerCase();
  const commit: InputWidgetCommit =
    commitRaw === 'auto' || commitRaw === 'blur' || commitRaw === 'button' ? commitRaw : 'enter';

  return {
    inputType,
    placeholder: toString(base.placeholder, ''),
    commit,
    buttonLabel: toString(base.buttonLabel, 'Set'),
    min: toString(base.min, ''),
    max: toString(base.max, ''),
    step: toString(base.step, ''),
    pattern: toString(base.pattern, ''),
    autocomplete: toString(base.autocomplete, ''),
    spellcheck: toBoolean(base.spellcheck, false)
  };
}

export function readTextAreaConfig(raw: WidgetConfigRecord | undefined): TextAreaWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const commitRaw = toString(base.commit, 'button').toLowerCase();
  const commit: TextAreaWidgetCommit =
    commitRaw === 'auto' || commitRaw === 'ctrl-enter' || commitRaw === 'blur' ? commitRaw : 'button';

  const maxLengthRaw = toNumber(base.maxLength);
  const maxLength =
    maxLengthRaw === null || maxLengthRaw <= 0 ? null : Math.max(1, Math.floor(maxLengthRaw));

  return {
    placeholder: toString(base.placeholder, ''),
    rows: Math.max(2, Math.min(16, Math.floor(toNumber(base.rows) ?? 4))),
    maxLength,
    commit,
    buttonLabel: toString(base.buttonLabel, 'Set'),
    spellcheck: toBoolean(base.spellcheck, true)
  };
}

export function readToggleConfig(raw: WidgetConfigRecord | undefined): ToggleWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const styleRaw = toString(base.style, 'switch').toLowerCase();
  const style: ToggleWidgetStyle = styleRaw === 'button' ? 'button' : 'switch';

  return {
    trueLabel: toString(base.trueLabel, 'On'),
    falseLabel: toString(base.falseLabel, 'Off'),
    style
  };
}

export function defaultDropdownOptions(signal: SignalRow): DropdownOption[] {
  if (signal.signal_type === 'bool') {
    return [
      { label: 'True', value: 'true' },
      { label: 'False', value: 'false' }
    ];
  }

  const raw = signal.value.trim();
  const chunks = raw.includes('|')
    ? raw.split('|')
    : raw.includes(',')
      ? raw.split(',')
      : [];

  const parsed = chunks
    .map((chunk) => chunk.trim())
    .filter((chunk) => chunk.length > 0)
    .slice(0, 20)
    .map((chunk) => ({ label: chunk, value: chunk }));

  if (parsed.length > 0) return parsed;

  if (raw) {
    return [{ label: raw, value: raw }];
  }

  return [];
}

export function parseDropdownOptionsInput(raw: string): DropdownOption[] {
  return raw
    .split(',')
    .map((chunk) => chunk.trim())
    .filter((chunk) => chunk.length > 0)
    .map((chunk) => {
      const pivot = chunk.indexOf(':');
      if (pivot <= 0) {
        return { label: chunk, value: chunk };
      }

      const label = chunk.slice(0, pivot).trim();
      const value = chunk.slice(pivot + 1).trim();
      return {
        label: label || value,
        value: value || label
      };
    });
}

export function formatDropdownOptions(options: DropdownOption[]): string {
  return options.map((option) => `${option.label}:${option.value}`).join(', ');
}

export function readDropdownConfig(
  raw: WidgetConfigRecord | undefined,
  signal: SignalRow
): DropdownWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const options = Array.isArray(base.options)
    ? base.options
        .map((entry) => {
          if (!isObject(entry)) return null;
          const label = toString(entry.label, '').trim();
          const value = toString(entry.value, '').trim();
          if (!label && !value) return null;
          return {
            label: label || value,
            value: value || label
          };
        })
        .filter((entry): entry is DropdownOption => entry !== null)
    : [];

  const commitRaw = toString(base.commit, 'button').toLowerCase();
  const commit: DropdownWidgetCommit = commitRaw === 'auto' ? 'auto' : 'button';

  return {
    options: options.length > 0 ? options : defaultDropdownOptions(signal),
    commit,
    buttonLabel: toString(base.buttonLabel, 'Set')
  };
}

export function readChoiceConfig(
  raw: WidgetConfigRecord | undefined,
  signal: SignalRow
): ChoiceWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const options = Array.isArray(base.options)
    ? base.options
        .map((entry) => {
          if (!isObject(entry)) return null;
          const label = toString(entry.label, '').trim();
          const value = toString(entry.value, '').trim();
          if (!label && !value) return null;
          return {
            label: label || value,
            value: value || label
          };
        })
        .filter((entry): entry is DropdownOption => entry !== null)
    : [];

  const directionRaw = toString(base.direction, 'vertical').toLowerCase();
  const direction: ChoiceWidgetDirection = directionRaw === 'horizontal' ? 'horizontal' : 'vertical';

  const commitRaw = toString(base.commit, 'auto').toLowerCase();
  const commit: ChoiceWidgetCommit = commitRaw === 'button' ? 'button' : 'auto';

  return {
    options: options.length > 0 ? options : defaultDropdownOptions(signal),
    direction,
    commit,
    buttonLabel: toString(base.buttonLabel, 'Apply')
  };
}
