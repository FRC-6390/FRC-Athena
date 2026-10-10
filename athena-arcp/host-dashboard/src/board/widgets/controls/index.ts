import { registerWidget } from '../../../lib/board/registry';
import type { SettingField } from '../../../lib/board/types';
import { chooserScore } from '../../../lib/board/widgets/controls-choice';
import { gainScore } from '../../../lib/board/widgets/controls-pid';
import { BOOL, NUMBER } from '../core';
import NumberInputWidget from './NumberInputWidget.svelte';
import TextInputWidget from './TextInputWidget.svelte';
import DropdownWidget from './DropdownWidget.svelte';
import SegmentedWidget from './SegmentedWidget.svelte';
import ButtonGroupWidget from './ButtonGroupWidget.svelte';
import HoldConfirmWidget from './HoldConfirmWidget.svelte';
import NudgeWidget from './NudgeWidget.svelte';
import GaugeWidget from './GaugeWidget.svelte';
import BarMeterWidget from './BarMeterWidget.svelte';
import StatTileWidget from './StatTileWidget.svelte';
import TextDisplayWidget from './TextDisplayWidget.svelte';
import BooleanMatrixWidget from './BooleanMatrixWidget.svelte';
import StatusListWidget from './StatusListWidget.svelte';
import PidTunerWidget from './PidTunerWidget.svelte';

const TONES = [
  { value: 'ok', label: 'Green' },
  { value: 'warn', label: 'Amber' },
  { value: 'fault', label: 'Red' },
  { value: 'info', label: 'Blue' },
  { value: 'off', label: 'Dark' }
];
const ALERT = [
  { value: 'high', label: 'Alarm when value rises to a threshold' },
  { value: 'low', label: 'Alarm when value falls to a threshold' }
];
const NONE = Number.NaN;
const isWritableNumber = (s: { writable: boolean; type: string }) => s.writable && NUMBER.includes(s.type);

/** Settings shared by the threshold-based value widgets. */
const alertSettings = (warn: number, fault: number): SettingField[] => [
  { key: 'warn', label: 'Warn at (blank = none)', kind: 'number', default: warn },
  { key: 'fault', label: 'Fault at (blank = none)', kind: 'number', default: fault },
  { key: 'alertWhen', label: 'Alarm direction', kind: 'enum', default: 'high', options: ALERT }
];

/** Registers the Controls and Values widget pack (see src/board/widgets/controls). */
export function registerControlWidgets(): void {
  registerWidget({
    type: 'number-input',
    name: 'Number input',
    category: 'Controls',
    description: 'Type a number and press Enter to send it. The − and + buttons step the draft. Shows the robot value next to your draft. Large changes ask first.',
    slots: [{ key: 'value', label: 'Number', matcher: { types: NUMBER, writable: true }, required: true }],
    settings: [
      { key: 'step', label: 'Step for − / +', kind: 'number', default: 1, min: 0, step: 0.1 },
      { key: 'min', label: 'Min (blank = none)', kind: 'number', default: NONE },
      { key: 'max', label: 'Max (blank = none)', kind: 'number', default: NONE }
    ],
    defaultSize: { w: 4, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => (isWritableNumber(s) ? 30 : 0),
    component: NumberInputWidget
  });

  registerWidget({
    type: 'text-input',
    name: 'Text input',
    category: 'Controls',
    description: 'Type text and press Enter to write a string to the robot. Shows the current robot value.',
    slots: [{ key: 'value', label: 'Text', matcher: { types: ['string'], writable: true }, required: true }],
    settings: [{ key: 'placeholder', label: 'Placeholder', kind: 'text', default: '' }],
    defaultSize: { w: 4, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => (s.type === 'string' && s.writable ? 35 : 0),
    component: TextInputWidget
  });

  const choiceSlots = [
    { key: 'value', label: 'Choice', matcher: { types: ['string'], writable: true }, required: true },
    { key: 'options', label: 'Options (string list, optional)', matcher: { types: ['string[]'] } }
  ];
  const choiceSettings: SettingField[] = [
    { key: 'choices', label: 'Choices (comma list, used without an options signal)', kind: 'text', default: '', placeholder: 'auto1, auto2, auto3' }
  ];

  registerWidget({
    type: 'dropdown',
    name: 'Dropdown',
    category: 'Controls',
    description: 'Pick one string from a list and send it. Options come from a bound string list, or from the Choices setting.',
    slots: choiceSlots,
    settings: choiceSettings,
    defaultSize: { w: 4, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => chooserScore(s),
    component: DropdownWidget
  });

  registerWidget({
    type: 'segmented',
    name: 'Segmented chooser',
    category: 'Controls',
    description: 'Pick one string from buttons laid out in a row. Same data as the dropdown; good for 2 to 5 modes.',
    slots: choiceSlots,
    settings: choiceSettings,
    defaultSize: { w: 6, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => {
      const score = chooserScore(s);
      return score > 0 ? score - 5 : 0;
    },
    component: SegmentedWidget
  });

  registerWidget({
    type: 'button-group',
    name: 'Button group',
    category: 'Controls',
    description: 'Several robot Actions as one grid of run buttons. Each shows its own running state; press a running one to stop it.',
    slots: [{ key: 'actions', label: 'Actions', matcher: { action: true }, multiple: true, required: true }],
    settings: [{ key: 'columns', label: 'Columns (0 = auto)', kind: 'number', default: 0, min: 0, max: 8, step: 1 }],
    defaultSize: { w: 6, h: 3 },
    minSize: { w: 3, h: 2 },
    score: (s) => (s.role === 'action' || s.localPath.startsWith('Actions/') ? 30 : 0),
    component: ButtonGroupWidget
  });

  registerWidget({
    type: 'hold-confirm',
    name: 'Hold to confirm',
    category: 'Controls',
    description: 'Press and hold for a set time to run a dangerous action. A fill shows progress; letting go early sends nothing.',
    slots: [{ key: 'action', label: 'Action', matcher: { action: true }, required: true }],
    settings: [
      { key: 'label', label: 'Label (blank = action name)', kind: 'text', default: '' },
      { key: 'holdMs', label: 'Hold time', kind: 'number', default: 1500, min: 200, max: 10000, step: 100, unit: 'ms' },
      { key: 'tone', label: 'Colour', kind: 'enum', default: 'warn', options: TONES.filter((t) => ['warn', 'fault', 'info'].includes(t.value)) }
    ],
    defaultSize: { w: 4, h: 2 },
    minSize: { w: 2, h: 2 },
    score: (s) => (s.role === 'action' || s.localPath.startsWith('Actions/') ? 15 : 0),
    component: HoldConfirmWidget
  });

  registerWidget({
    type: 'nudge',
    name: 'Nudge buttons',
    category: 'Controls',
    description: 'Current value between − and + buttons. Each press sends value ± step. Steps stay inside a safe range unless you set Min and Max.',
    slots: [{ key: 'value', label: 'Number', matcher: { types: NUMBER, writable: true }, required: true }],
    settings: [
      { key: 'step', label: 'Step', kind: 'number', default: 1, min: 0, step: 0.1 },
      { key: 'min', label: 'Min (blank = safe default)', kind: 'number', default: NONE },
      { key: 'max', label: 'Max (blank = safe default)', kind: 'number', default: NONE }
    ],
    defaultSize: { w: 5, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => (isWritableNumber(s) ? 25 : 0),
    component: NudgeWidget
  });

  registerWidget({
    type: 'gauge',
    name: 'Gauge',
    category: 'Values',
    description: 'Radial gauge for one number, with amber and red bands from the warn and fault thresholds.',
    slots: [{ key: 'value', label: 'Number', matcher: { types: NUMBER }, required: true }],
    settings: [
      { key: 'min', label: 'Scale min', kind: 'number', default: 0 },
      { key: 'max', label: 'Scale max', kind: 'number', default: 100 },
      ...alertSettings(NONE, NONE)
    ],
    defaultSize: { w: 4, h: 4 },
    minSize: { w: 3, h: 3 },
    score: (s) => (NUMBER.includes(s.type) ? (s.writable ? 15 : 35) : 0),
    component: GaugeWidget
  });

  registerWidget({
    type: 'bar-meter',
    name: 'Bar meter',
    category: 'Values',
    description: 'Horizontal or vertical bar for one number, with thresholds and an optional target marker from a second signal.',
    slots: [
      { key: 'value', label: 'Number', matcher: { types: NUMBER }, required: true },
      { key: 'target', label: 'Target marker (optional)', matcher: { types: NUMBER } }
    ],
    settings: [
      { key: 'orientation', label: 'Orientation', kind: 'enum', default: 'horizontal', options: [{ value: 'horizontal', label: 'Horizontal' }, { value: 'vertical', label: 'Vertical' }] },
      { key: 'min', label: 'Scale min', kind: 'number', default: 0 },
      { key: 'max', label: 'Scale max', kind: 'number', default: 100 },
      ...alertSettings(NONE, NONE)
    ],
    defaultSize: { w: 6, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => (NUMBER.includes(s.type) ? (s.writable ? 15 : 30) : 0),
    component: BarMeterWidget
  });

  registerWidget({
    type: 'stat-tile',
    name: 'Stat tile',
    category: 'Values',
    description: 'Big number with a sparkline of the last N seconds and the change over that window.',
    slots: [{ key: 'value', label: 'Number', matcher: { types: NUMBER }, required: true }],
    settings: [
      { key: 'window', label: 'Window', kind: 'number', default: 30, min: 1, max: 600, step: 1, unit: 's' },
      { key: 'decimals', label: 'Decimals (-1 = auto)', kind: 'number', default: -1, min: -1, max: 6, step: 1 },
      { key: 'unit', label: 'Unit override', kind: 'text', default: '' },
      { key: 'showDelta', label: 'Show change over window', kind: 'bool', default: true }
    ],
    defaultSize: { w: 4, h: 3 },
    minSize: { w: 3, h: 2 },
    score: (s) => (NUMBER.includes(s.type) ? (s.writable ? 20 : 38) : 0),
    component: StatTileWidget
  });

  registerWidget({
    type: 'text-display',
    name: 'Text display',
    category: 'Values',
    description: 'A string signal shown in full, with wrapping. Use monospace for logs and paths.',
    slots: [{ key: 'value', label: 'Text', matcher: { types: ['string'] }, required: true }],
    settings: [
      { key: 'mono', label: 'Monospace', kind: 'bool', default: false },
      { key: 'maxLines', label: 'Max lines (0 = no limit)', kind: 'number', default: 0, min: 0, max: 20, step: 1 }
    ],
    defaultSize: { w: 6, h: 2 },
    minSize: { w: 2, h: 1 },
    score: (s) => (s.type === 'string' ? (s.writable ? 20 : 35) : 0),
    component: TextDisplayWidget
  });

  registerWidget({
    type: 'boolean-matrix',
    name: 'Boolean matrix',
    category: 'Values',
    description: 'Many true/false signals as a grid of labelled lights. Good for sensor and limit-switch panels.',
    slots: [{ key: 'bits', label: 'Booleans', matcher: { types: BOOL }, multiple: true, required: true }],
    settings: [
      { key: 'columns', label: 'Columns (0 = auto)', kind: 'number', default: 0, min: 0, max: 12, step: 1 },
      { key: 'trueTone', label: 'Colour when true', kind: 'enum', default: 'ok', options: TONES },
      { key: 'falseTone', label: 'Colour when false', kind: 'enum', default: 'off', options: TONES }
    ],
    defaultSize: { w: 6, h: 3 },
    minSize: { w: 3, h: 2 },
    score: (s) => (s.type === 'bool' ? 20 : 0),
    component: BooleanMatrixWidget
  });

  registerWidget({
    type: 'status-list',
    name: 'Status list',
    category: 'Values',
    description: 'Several signals in one compact list. Each row gets a tone from the thresholds (numbers) or true/false colours (booleans).',
    slots: [{ key: 'rows', label: 'Signals', matcher: {}, multiple: true, required: true }],
    settings: [
      { key: 'warnAbove', label: 'Warn at or above', kind: 'number', default: NONE },
      { key: 'faultAbove', label: 'Fault at or above', kind: 'number', default: NONE },
      { key: 'warnBelow', label: 'Warn at or below', kind: 'number', default: NONE },
      { key: 'faultBelow', label: 'Fault at or below', kind: 'number', default: NONE },
      { key: 'trueTone', label: 'Colour when true', kind: 'enum', default: 'ok', options: TONES },
      { key: 'falseTone', label: 'Colour when false', kind: 'enum', default: 'off', options: TONES }
    ],
    defaultSize: { w: 5, h: 4 },
    minSize: { w: 3, h: 2 },
    score: () => 10,
    component: StatusListWidget
  });

  registerWidget({
    type: 'pid-tuner',
    name: 'PID tuner',
    category: 'Controls',
    description: 'Table of writable gains (kP, kI, kD, ...). Edit each one inline. Restore uses the sibling Restore action when there is one.',
    slots: [
      { key: 'gains', label: 'Gains', matcher: { types: NUMBER, writable: true }, multiple: true, required: true },
      { key: 'restore', label: 'Restore action (optional, found automatically)', matcher: { action: true } }
    ],
    settings: [],
    defaultSize: { w: 5, h: 5 },
    minSize: { w: 4, h: 3 },
    score: (s) => gainScore(s),
    component: PidTunerWidget
  });
}
