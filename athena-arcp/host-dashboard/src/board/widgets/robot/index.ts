import { registerWidget } from '../../../lib/board/registry';
import type { SignalDescriptor } from '../../../lib/data/types';
import type { BindingSlot, SettingField, SignalMatcher, WidgetDef } from '../../../lib/board/types';
import { controlRoot } from '../../../lib/board/widgets/robot-siblings';
import { NUMBER } from '../core';
import XYWidget from './XYWidget.svelte';
import RollingStatsWidget from './RollingStatsWidget.svelte';
import StripWidget from './StripWidget.svelte';
import CompassWidget from './CompassWidget.svelte';
import AngleDialWidget from './AngleDialWidget.svelte';
import ArmWidget from './ArmWidget.svelte';
import ElevatorWidget from './ElevatorWidget.svelte';
import SwerveWidget from './SwerveWidget.svelte';
import MotorCardWidget from './MotorCardWidget.svelte';
import MechanismCardWidget from './MechanismCardWidget.svelte';
import ControlTunerWidget from './ControlTunerWidget.svelte';
import PowerSplitWidget from './PowerSplitWidget.svelte';

const isNumber = (s: SignalDescriptor) => s.type === 'f64' || s.type === 'i64';
/** Name-based suggestion weight: a dropped signal whose name says what the widget shows scores higher. */
const named = (s: SignalDescriptor, pattern: RegExp, hit: number, miss: number) => (pattern.test(s.name) || pattern.test(s.path) ? hit : miss);

const num = (key: string, label: string, matcher: SignalMatcher = { types: NUMBER }, required = false): BindingSlot => ({
  key,
  label,
  matcher,
  required
});

const angleUnit = (key = 'unit'): SettingField => ({
  key,
  label: 'Angle unit',
  kind: 'enum',
  default: 'deg',
  options: [
    { value: 'deg', label: 'Degrees' },
    { value: 'rad', label: 'Radians' }
  ]
});

const windowField = (def = 10): SettingField => ({ key: 'window', label: 'Window', kind: 'number', default: def, min: 1, max: 120, step: 1, unit: 's' });

const moduleSlots: BindingSlot[] = ['fl', 'fr', 'bl', 'br'].flatMap((m) => [
  num(`${m}_speed`, `${m.toUpperCase()} speed`),
  num(`${m}_angle`, `${m.toUpperCase()} angle`)
]);

export function registerRobotWidgets(): void {
  const defs: WidgetDef[] = [
    {
      type: 'xy',
      name: 'XY plot',
      category: 'Charts',
      description: 'Plots one signal against another over a window, with a fading trail. Good for paths and pose.',
      slots: [num('x', 'X', { types: NUMBER }, true), num('y', 'Y', { types: NUMBER }, true)],
      settings: [
        windowField(10),
        { key: 'trail', label: 'Fading trail', kind: 'bool', default: true },
        { key: 'equal', label: 'Equal axis scale', kind: 'bool', default: false }
      ],
      defaultSize: { w: 5, h: 5 },
      minSize: { w: 3, h: 3 },
      score: (s) => (isNumber(s) ? 25 : 0),
      component: XYWidget
    },
    {
      type: 'rollingStats',
      name: 'Rolling stats',
      category: 'Charts',
      description: 'Min, max, mean, standard deviation and sample rate of a number over a window, with a histogram.',
      slots: [num('value', 'Signal', { types: NUMBER }, true)],
      settings: [
        windowField(10),
        { key: 'bins', label: 'Histogram bins', kind: 'number', default: 12, min: 2, max: 32, step: 1 }
      ],
      defaultSize: { w: 5, h: 3 },
      minSize: { w: 3, h: 2 },
      score: (s) => (isNumber(s) ? 25 : 0),
      component: RollingStatsWidget
    },
    {
      type: 'strip',
      name: 'Strip chart',
      category: 'Charts',
      description: 'Several numbers stacked as lanes with their own scales and one shared time axis. Use it for signals with different units.',
      slots: [num('series', 'Series', { types: NUMBER }, true)],
      settings: [windowField(10)],
      defaultSize: { w: 8, h: 6 },
      minSize: { w: 4, h: 3 },
      score: (s) => (isNumber(s) ? 30 : 0),
      component: StripWidget
    },
    {
      type: 'compass',
      name: 'Compass',
      category: 'Robot',
      description: 'Heading as a rotating dial with the robot pointer at the top. Optional target heading shown as a marker.',
      slots: [num('heading', 'Heading', { types: NUMBER }, true), num('target', 'Target')],
      settings: [angleUnit(), { key: 'offset', label: 'Offset', kind: 'number', default: 0, unit: '°' }],
      defaultSize: { w: 4, h: 4 },
      minSize: { w: 2, h: 2 },
      score: (s) => (isNumber(s) ? named(s, /heading|yaw|gyro/i, 60, 10) : 0),
      component: CompassWidget
    },
    {
      type: 'angleDial',
      name: 'Angle dial',
      category: 'Robot',
      description: 'Turret or joint position on a circular dial, with the allowed travel shaded and an optional target marker.',
      slots: [num('angle', 'Angle', { types: NUMBER }, true), num('target', 'Target')],
      settings: [
        angleUnit(),
        { key: 'minAngle', label: 'Minimum angle', kind: 'number', default: -170, unit: '°' },
        { key: 'maxAngle', label: 'Maximum angle', kind: 'number', default: 170, unit: '°' }
      ],
      defaultSize: { w: 4, h: 4 },
      minSize: { w: 2, h: 2 },
      score: (s) => (isNumber(s) ? named(s, /angle|position|turret|rotation/i, 40, 10) : 0),
      component: AngleDialWidget
    },
    {
      type: 'arm',
      name: 'Arm',
      category: 'Robot',
      description: 'Draws one or two joint angles as a 2D arm. Lengths are in metres; the elbow is relative to the upper link by default.',
      slots: [num('shoulder', 'Shoulder', { types: NUMBER }, true), num('elbow', 'Elbow')],
      settings: [
        angleUnit(),
        { key: 'upper', label: 'Upper link', kind: 'number', default: 0.5, min: 0.01, step: 0.01, unit: 'm' },
        { key: 'lower', label: 'Lower link', kind: 'number', default: 0.5, min: 0.01, step: 0.01, unit: 'm' },
        { key: 'elbowRelative', label: 'Elbow relative to upper link', kind: 'bool', default: true }
      ],
      defaultSize: { w: 4, h: 4 },
      minSize: { w: 3, h: 3 },
      score: (s) => (isNumber(s) ? named(s, /shoulder|elbow|arm|pivot/i, 50, 10) : 0),
      component: ArmWidget
    },
    {
      type: 'elevator',
      name: 'Elevator',
      category: 'Robot',
      description: 'Height on a vertical shaft between its limits, with a carriage and optional target marker.',
      slots: [num('height', 'Height', { types: NUMBER }, true), num('target', 'Target')],
      settings: [
        { key: 'min', label: 'Bottom height', kind: 'number', default: 0, step: 0.01 },
        { key: 'max', label: 'Top height', kind: 'number', default: 1.5, step: 0.01 },
        { key: 'unit', label: 'Unit', kind: 'text', default: 'm' }
      ],
      defaultSize: { w: 3, h: 5 },
      minSize: { w: 2, h: 3 },
      score: (s) => (isNumber(s) ? named(s, /height|elevator|extension|lift/i, 50, 10) : 0),
      component: ElevatorWidget
    },
    {
      type: 'swerve',
      name: 'Swerve drive',
      category: 'Robot',
      description: 'Four swerve modules as wheels with speed vectors, plus robot heading and the chassis velocity. Takes an f64[] of speed and angle pairs (FL, FR, BL, BR) or individual module slots.',
      slots: [
        { key: 'states', label: 'Module states (f64[])', matcher: { types: ['f64[]'] } },
        num('heading', 'Heading'),
        ...moduleSlots
      ],
      settings: [
        { key: 'angleUnit', label: 'Module and heading unit', kind: 'enum', default: 'rad', options: [{ value: 'rad', label: 'Radians' }, { value: 'deg', label: 'Degrees' }] },
        { key: 'wheelbase', label: 'Wheelbase (front to back)', kind: 'number', default: 0.6, min: 0.1, step: 0.01, unit: 'm' },
        { key: 'trackwidth', label: 'Track width (left to right)', kind: 'number', default: 0.5, min: 0.1, step: 0.01, unit: 'm' },
        { key: 'maxSpeed', label: 'Speed for full-length vector', kind: 'number', default: 4, min: 0.1, step: 0.1, unit: 'm/s' }
      ],
      defaultSize: { w: 5, h: 5 },
      minSize: { w: 3, h: 4 },
      score: (s) => (s.type === 'f64[]' && /state|module|swerve/i.test(s.path) ? 60 : 0),
      component: SwerveWidget
    },
    {
      type: 'motorCard', titleFrom: 'device',
      name: 'Motor card',
      category: 'Robot',
      description: 'A motor or device group from one of its signals: current, voltage, temperature, derate and link state. Siblings are found by path.',
      slots: [{ key: 'signal', label: 'Device signal', matcher: { test: (s) => s.path.includes('/Devices/') }, required: true }],
      settings: [
        { key: 'tempWarn', label: 'Warn above', kind: 'number', default: 60, unit: '°C' },
        { key: 'tempFault', label: 'Fault above', kind: 'number', default: 80, unit: '°C' }
      ],
      defaultSize: { w: 4, h: 4 },
      minSize: { w: 3, h: 3 },
      score: (s) => (s.path.includes('/Devices/') ? (/temperature|current|derate/i.test(s.name) ? 70 : 60) : 0),
      component: MotorCardWidget
    },
    {
      type: 'mechanismCard', titleFrom: 'mechanism',
      name: 'Mechanism card',
      category: 'Robot',
      description: 'A mechanism summary from any of its signals: State and Reason (coloured by tone), stage, active actions and faults.',
      slots: [{ key: 'signal', label: 'Mechanism signal', matcher: { test: (s) => s.mechanismPath !== '' }, required: true }],
      settings: [],
      defaultSize: { w: 4, h: 3 },
      minSize: { w: 3, h: 2 },
      score: (s) => (s.mechanismPath === '' ? 0 : s.localPath.startsWith('State/') ? 40 : 20),
      component: MechanismCardWidget
    },
    {
      type: 'controlTuner', titleFrom: 'control',
      name: 'Control tuner',
      category: 'Robot',
      description: 'A control loop from any of its signals: target, reference and measured traces with the gains editable inline.',
      slots: [{ key: 'signal', label: 'Control signal', matcher: { test: (s) => controlRoot(s.path) !== null }, required: true }],
      settings: [windowField(10)],
      defaultSize: { w: 6, h: 5 },
      minSize: { w: 4, h: 4 },
      score: (s) => (controlRoot(s.path) === null ? 0 : /target|reference|measured|feedback|^k[A-Z]/i.test(s.name) ? 55 : 35),
      component: ControlTunerWidget
    },
    {
      type: 'powerSplit', titleFrom: 'fixed',
      name: 'Power split',
      category: 'Robot',
      description: 'Several current signals as one stacked bar and a list, with the total amps. Drop more channels onto the Currents slot.',
      slots: [num('currents', 'Currents', { types: NUMBER }, true)],
      settings: [{ key: 'channels', label: 'Show channel list', kind: 'bool', default: true }],
      defaultSize: { w: 5, h: 3 },
      minSize: { w: 3, h: 2 },
      score: (s) => (isNumber(s) ? named(s, /current|power|amps/i, 40, 10) : 0),
      component: PowerSplitWidget
    }
  ];
  for (const def of defs) registerWidget(def);
}
