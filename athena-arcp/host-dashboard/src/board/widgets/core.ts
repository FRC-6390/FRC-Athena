import { registerWidget } from '../../lib/board/registry';
import type { SignalDescriptor } from '../../lib/data/types';
import ValueWidget from './ValueWidget.svelte';
import GraphWidget from './GraphWidget.svelte';
import BooleanWidget from './BooleanWidget.svelte';
import ToggleWidget from './ToggleWidget.svelte';
import ActionWidget from './ActionWidget.svelte';
import SliderWidget from './SliderWidget.svelte';

/** Signal type families shared by widget registrations. */
export const NUMBER = ['f64', 'i64'];
export const BOOL = ['bool'];
export const isAction = (s: SignalDescriptor) => s.role === 'action' || s.localPath.startsWith('Actions/');

const toneOptions = [
  { value: 'ok', label: 'Green' },
  { value: 'warn', label: 'Amber' },
  { value: 'fault', label: 'Red' },
  { value: 'info', label: 'Blue' },
  { value: 'off', label: 'Dark' }
];

export function registerCoreWidgets(): void {
  registerWidget({
    type: 'value',
    name: 'Value',
    category: 'Values',
    description: 'A single value, scaled to the tile. Click to set writable values.',
    slots: [{ key: 'value', label: 'Signal', matcher: {}, required: true }],
    settings: [
      { key: 'decimals', label: 'Decimals (-1 = auto)', kind: 'number', default: -1, min: -1, max: 6, step: 1 },
      { key: 'unit', label: 'Unit override', kind: 'text', default: '' },
      { key: 'warnAbove', label: 'Warn at or above', kind: 'number', default: Number.NaN },
      { key: 'faultAbove', label: 'Fault at or above', kind: 'number', default: Number.NaN }
    ],
    defaultSize: { w: 4, h: 2 },
    minSize: { w: 2, h: 1 },
    score: (s) => (s.type === 'f64' || s.type === 'i64' ? 50 : s.type === 'string' ? 40 : 10),
    component: ValueWidget
  });

  registerWidget({
    type: 'graph',
    name: 'Graph',
    category: 'Charts',
    description: 'Live line graph of one or more numeric signals over a rolling window.',
    slots: [{ key: 'series', label: 'Series', matcher: { types: NUMBER }, multiple: true, required: true }],
    settings: [
      { key: 'window', label: 'Window', kind: 'number', default: 10, min: 1, max: 120, step: 1, unit: 's' },
      { key: 'min', label: 'Y min (blank = auto)', kind: 'number', default: Number.NaN },
      { key: 'max', label: 'Y max (blank = auto)', kind: 'number', default: Number.NaN },
      { key: 'legend', label: 'Legend', kind: 'bool', default: true }
    ],
    defaultSize: { w: 8, h: 5 },
    minSize: { w: 3, h: 2 },
    score: (s) => (s.writable ? 20 : 45),
    component: GraphWidget
  });

  registerWidget({
    type: 'boolean',
    name: 'Indicator light',
    category: 'Values',
    description: 'A large coloured light for a true/false signal.',
    slots: [{ key: 'value', label: 'Signal', matcher: { types: BOOL }, required: true }],
    settings: [
      { key: 'trueText', label: 'Text when true', kind: 'text', default: 'Yes' },
      { key: 'falseText', label: 'Text when false', kind: 'text', default: 'No' },
      { key: 'trueTone', label: 'Colour when true', kind: 'enum', default: 'ok', options: toneOptions },
      { key: 'falseTone', label: 'Colour when false', kind: 'enum', default: 'off', options: toneOptions }
    ],
    defaultSize: { w: 3, h: 2 },
    minSize: { w: 1, h: 1 },
    score: (s) => (s.writable ? 20 : 50),
    component: BooleanWidget
  });

  registerWidget({
    type: 'toggle',
    name: 'Toggle switch',
    category: 'Controls',
    description: 'Switch that writes a boolean to the robot.',
    slots: [{ key: 'value', label: 'Signal', matcher: { types: BOOL, writable: true }, required: true }],
    settings: [
      { key: 'onText', label: 'On text', kind: 'text', default: 'On' },
      { key: 'offText', label: 'Off text', kind: 'text', default: 'Off' }
    ],
    defaultSize: { w: 3, h: 2 },
    minSize: { w: 2, h: 1 },
    score: () => 55,
    component: ToggleWidget
  });

  registerWidget({
    type: 'action',
    name: 'Action button',
    category: 'Controls',
    description: 'Runs a robot Action: tap, toggle, or hold to run.',
    slots: [{ key: 'action', label: 'Action', matcher: { test: isAction }, required: true }],
    settings: [
      { key: 'label', label: 'Label', kind: 'text', default: '' },
      {
        key: 'mode',
        label: 'Behaviour',
        kind: 'enum',
        default: 'tap',
        options: [
          { value: 'tap', label: 'Tap to run' },
          { value: 'toggle', label: 'Tap to start / stop' },
          { value: 'hold', label: 'Hold to run' }
        ]
      }
    ],
    defaultSize: { w: 4, h: 2 },
    minSize: { w: 2, h: 1 },
    score: () => 60,
    component: ActionWidget
  });

  registerWidget({
    type: 'slider',
    name: 'Slider',
    category: 'Controls',
    description: 'Drag to set a writable number; shows the robot value while dragging.',
    slots: [{ key: 'value', label: 'Signal', matcher: { types: NUMBER, writable: true }, required: true }],
    settings: [
      { key: 'min', label: 'Min', kind: 'number', default: 0 },
      { key: 'max', label: 'Max', kind: 'number', default: 100 },
      { key: 'step', label: 'Step', kind: 'number', default: 1 },
      { key: 'sendWhileDragging', label: 'Send while dragging', kind: 'bool', default: false }
    ],
    defaultSize: { w: 6, h: 2 },
    minSize: { w: 3, h: 2 },
    score: () => 45,
    component: SliderWidget
  });
}
