import { registerWidget } from '../../../lib/board/registry';
import type { SignalDescriptor } from '../../../lib/data/types';
import { isJoystickPath } from '../../../lib/board/widgets/match-controller';
import { ALLIANCE_TRUE_OPTIONS, NUMBER, isBoolean, isNumeric, isString, nameMatches } from './shared';
import FieldWidget from './FieldWidget.svelte';
import TimerWidget from './TimerWidget.svelte';
import AllianceWidget from './AllianceWidget.svelte';
import BatteryWidget from './BatteryWidget.svelte';
import AlertsWidget from './AlertsWidget.svelte';
import ChooserWidget from './ChooserWidget.svelte';
import CameraWidget from './CameraWidget.svelte';
import ControllerWidget from './ControllerWidget.svelte';
import LogWidget from './LogWidget.svelte';
import LabelWidget from './LabelWidget.svelte';
import NoteWidget from './NoteWidget.svelte';
import ImageWidget from './ImageWidget.svelte';
import ChecklistWidget from './ChecklistWidget.svelte';
import StopwatchWidget from './StopwatchWidget.svelte';

const POSE = ['struct:Pose2d'];

/** Registers the match and layout widget pack. Called once from `registerAllWidgets()`. */
export function registerMatchWidgets(): void {
  registerWidget({
    type: 'match-field', titleFrom: 'fixed',
    name: 'Field 2D',
    category: 'Match',
    description: 'Robot pose on the field with heading, trail and alliance-coloured bumpers. Extra Pose2d signals (targets, vision) are drawn as outlines.',
    slots: [
      { key: 'pose', label: 'Robot pose, then extra poses', matcher: { types: POSE }, multiple: true, required: true },
      { key: 'alliance', label: 'Alliance (optional)', matcher: { types: ['string', 'bool'] } }
    ],
    settings: [
      { key: 'trail', label: 'Trail length', kind: 'number', default: 5, min: 0, max: 60, step: 1, unit: 's' },
      {
        key: 'perspective',
        label: 'View from',
        kind: 'enum',
        default: 'blue',
        options: [
          { value: 'blue', label: 'Blue alliance wall' },
          { value: 'red', label: 'Red alliance wall' },
          { value: 'auto', label: 'Auto (follow alliance)' }
        ]
      },
      { key: 'trueIs', label: 'Boolean alliance: true means', kind: 'enum', default: 'red', options: ALLIANCE_TRUE_OPTIONS },
      { key: 'showImage', label: 'Show field image', kind: 'bool', default: true }
    ],
    defaultSize: { w: 6, h: 6 },
    minSize: { w: 3, h: 3 },
    score: (s) => (s.type === 'struct:Pose2d' ? 85 : 0),
    component: FieldWidget
  });

  registerWidget({
    type: 'match-timer', titleFrom: 'fixed',
    name: 'Match timer',
    category: 'Match',
    description: 'Big match clock with auto, teleop, endgame and final colours. Uses a match-time signal, or the match markers when none is bound.',
    slots: [{ key: 'time', label: 'Match time (optional)', matcher: { types: NUMBER } }],
    settings: [
      {
        key: 'source',
        label: 'Bound signal counts',
        kind: 'enum',
        default: 'period',
        options: [
          { value: 'period', label: 'Seconds left in period (WPILib match time)' },
          { value: 'elapsed', label: 'Seconds elapsed in match' },
          { value: 'remaining', label: 'Seconds left in match' }
        ]
      },
      { key: 'endgame', label: 'Endgame colour at', kind: 'number', default: 30, min: 1, max: 60, step: 1, unit: 's left' },
      { key: 'final', label: 'Final colour at', kind: 'number', default: 15, min: 1, max: 30, step: 1, unit: 's left' }
    ],
    defaultSize: { w: 4, h: 3 },
    minSize: { w: 2, h: 2 },
    score: (s) => (isNumeric(s) && nameMatches(s, /time/i) ? 80 : 10),
    component: TimerWidget
  });

  registerWidget({
    type: 'match-alliance', titleFrom: 'fixed',
    name: 'Alliance and station',
    category: 'Match',
    description: 'Alliance colour block with the driver station number. Bind an alliance (string or boolean) and a station (1 to 3).',
    slots: [
      { key: 'alliance', label: 'Alliance (optional)', matcher: { types: ['string', 'bool'] } },
      { key: 'station', label: 'Station (optional)', matcher: { types: NUMBER } }
    ],
    settings: [{ key: 'trueIs', label: 'Boolean alliance: true means', kind: 'enum', default: 'red', options: ALLIANCE_TRUE_OPTIONS }],
    defaultSize: { w: 5, h: 2 },
    minSize: { w: 3, h: 2 },
    score: (s) => (nameMatches(s, /alliance|station/i) ? 75 : 5),
    component: AllianceWidget
  });

  registerWidget({
    type: 'match-battery', titleFrom: 'fixed',
    name: 'Battery',
    category: 'Match',
    description: 'Battery voltage with a sag chart. Colours change below the warning and fault voltages.',
    slots: [{ key: 'volts', label: 'Voltage', matcher: { types: NUMBER }, required: true }],
    settings: [
      { key: 'warnBelow', label: 'Warn below', kind: 'number', default: 12, min: 0, max: 15, step: 0.1, unit: 'V' },
      { key: 'faultBelow', label: 'Fault below', kind: 'number', default: 11, min: 0, max: 15, step: 0.1, unit: 'V' },
      { key: 'window', label: 'Sag history', kind: 'number', default: 60, min: 5, max: 600, step: 5, unit: 's' }
    ],
    defaultSize: { w: 4, h: 4 },
    minSize: { w: 2, h: 3 },
    score: (s) => (isNumeric(s) && nameMatches(s, /battery|volt/i) ? 90 : isNumeric(s) ? 15 : 0),
    component: BatteryWidget
  });

  registerWidget({
    type: 'match-alerts', titleFrom: 'fixed',
    name: 'Alerts',
    category: 'Match',
    description: 'Newest faults, stage changes and checks from the robot, plus any boolean fault signals bound here. Dismiss on this screen only.',
    slots: [{ key: 'faults', label: 'Fault signals (true = fault)', matcher: { types: ['bool'] }, multiple: true }],
    settings: [
      { key: 'count', label: 'Entries shown', kind: 'number', default: 10, min: 1, max: 50, step: 1 },
      { key: 'showWarnings', label: 'Show stage changes', kind: 'bool', default: true }
    ],
    defaultSize: { w: 5, h: 4 },
    minSize: { w: 3, h: 3 },
    score: (s: SignalDescriptor) => (isBoolean(s) && nameMatches(s, /fault|error|fail/i) ? 70 : isBoolean(s) ? 15 : 0),
    component: AlertsWidget
  });

  registerWidget({
    type: 'match-chooser',
    name: 'Auto chooser',
    category: 'Match',
    description: 'Big buttons to pick the autonomous routine. Options come from a bound string list or from the settings (separated by |).',
    slots: [
      { key: 'selected', label: 'Selected (writable string)', matcher: { types: ['string'], writable: true }, required: true },
      { key: 'options', label: 'Options list (optional)', matcher: { types: ['string[]'] } }
    ],
    settings: [{ key: 'options', label: 'Options (separate with |)', kind: 'text', default: '', placeholder: 'Left | Centre | Right' }],
    defaultSize: { w: 6, h: 3 },
    minSize: { w: 3, h: 2 },
    score: (s) => (s.writable && isString(s) ? 80 : s.type === 'string[]' ? 60 : 0),
    component: ChooserWidget
  });

  registerWidget({
    type: 'match-camera',
    name: 'Camera',
    category: 'Match',
    description: 'MJPEG camera stream (PhotonVision, Limelight or a USB camera server) with an optional crosshair. Reconnects when the feed drops.',
    slots: [{ key: 'url', label: 'Stream URL (optional)', matcher: { types: ['string'] } }],
    settings: [
      { key: 'url', label: 'Stream URL', kind: 'text', default: 'http://photonvision.local:1182/stream.mjpg', placeholder: 'http://10.TE.AM.11:1182/stream.mjpg' },
      { key: 'crosshair', label: 'Show crosshair', kind: 'bool', default: false },
      {
        key: 'fit',
        label: 'Fit',
        kind: 'enum',
        default: 'contain',
        options: [
          { value: 'contain', label: 'Whole image' },
          { value: 'cover', label: 'Fill tile (crops)' }
        ]
      }
    ],
    defaultSize: { w: 6, h: 5 },
    minSize: { w: 3, h: 3 },
    score: (s) => (isString(s) && nameMatches(s, /stream|url/i) ? 60 : 0),
    component: CameraWidget
  });

  registerWidget({
    type: 'match-controller',
    name: 'Controller',
    category: 'Match',
    description: 'Gamepad view: face buttons, bumpers, triggers and sticks from input/joystick/<port>/ signals. Pressed controls light up.',
    slots: [{ key: 'input', label: 'Joystick signals (optional)', matcher: { test: (s) => isJoystickPath(s.path) }, multiple: true }],
    settings: [{ key: 'port', label: 'Joystick port', kind: 'number', default: 0, min: 0, max: 5, step: 1 }],
    defaultSize: { w: 4, h: 4 },
    minSize: { w: 3, h: 3 },
    score: (s) => (isJoystickPath(s.path) ? 60 : 0),
    component: ControllerWidget
  });

  registerWidget({
    type: 'match-log',
    name: 'Log feed',
    category: 'Match',
    description: 'History of a text signal (status messages, states) with the time of each change. Repeats are collapsed.',
    slots: [{ key: 'text', label: 'Text signal', matcher: { types: ['string'] }, required: true }],
    settings: [
      { key: 'lines', label: 'Lines kept', kind: 'number', default: 50, min: 5, max: 500, step: 5 },
      { key: 'showTime', label: 'Show time', kind: 'bool', default: true }
    ],
    defaultSize: { w: 5, h: 4 },
    minSize: { w: 3, h: 3 },
    score: (s) => (isString(s) && nameMatches(s, /status|message|state|log|reason/i) ? 65 : isString(s) ? 25 : 0),
    component: LogWidget
  });

  registerWidget({
    type: 'layout-label',
    name: 'Label',
    category: 'Layout',
    description: 'Section header or sign. Set text, size and alignment in settings.',
    slots: [],
    settings: [
      { key: 'text', label: 'Text', kind: 'text', default: 'Section' },
      {
        key: 'size',
        label: 'Size',
        kind: 'enum',
        default: 'large',
        options: [
          { value: 'auto', label: 'Fit tile' },
          { value: 'small', label: 'Small' },
          { value: 'medium', label: 'Medium' },
          { value: 'large', label: 'Large' },
          { value: 'huge', label: 'Huge' }
        ]
      },
      {
        key: 'align',
        label: 'Alignment',
        kind: 'enum',
        default: 'left',
        options: [
          { value: 'left', label: 'Left' },
          { value: 'center', label: 'Centre' },
          { value: 'right', label: 'Right' }
        ]
      },
      {
        key: 'tone',
        label: 'Text colour',
        kind: 'enum',
        default: 'strong',
        options: [
          { value: 'strong', label: 'Default' },
          { value: 'soft', label: 'Muted' },
          { value: 'brand', label: 'Athena red' },
          { value: 'info', label: 'Blue' },
          { value: 'ok', label: 'Green' },
          { value: 'warn', label: 'Amber' }
        ]
      }
    ],
    defaultSize: { w: 6, h: 2 },
    minSize: { w: 2, h: 1 },
    component: LabelWidget
  });

  registerWidget({
    type: 'layout-note',
    name: 'Note',
    category: 'Layout',
    description: 'Multi-line notes with headings (#), bullet lists (-) and **bold**.',
    slots: [],
    settings: [
      {
        key: 'text',
        label: 'Text',
        kind: 'textarea',
        rows: 8,
        default: '## Pit notes\n- Check bumpers before queueing\n- **Battery:** swap below 12.5 V'
      },
      {
        key: 'size',
        label: 'Text size',
        kind: 'enum',
        default: 'normal',
        options: [
          { value: 'normal', label: 'Normal' },
          { value: 'small', label: 'Small' }
        ]
      }
    ],
    defaultSize: { w: 6, h: 5 },
    minSize: { w: 3, h: 2 },
    component: NoteWidget
  });

  registerWidget({
    type: 'layout-image',
    name: 'Image',
    category: 'Layout',
    description: 'A picture from a web address, a data image, or a path on this dashboard (for example a team logo or a diagram).',
    slots: [],
    settings: [
      { key: 'url', label: 'Image URL', kind: 'text', default: '', placeholder: 'https://… or data:image/…' },
      { key: 'alt', label: 'Description', kind: 'text', default: '' },
      {
        key: 'fit',
        label: 'Fit',
        kind: 'enum',
        default: 'contain',
        options: [
          { value: 'contain', label: 'Whole image' },
          { value: 'cover', label: 'Fill tile (crops)' },
          { value: 'fill', label: 'Stretch' }
        ]
      }
    ],
    defaultSize: { w: 4, h: 4 },
    minSize: { w: 2, h: 2 },
    component: ImageWidget
  });

  registerWidget({
    type: 'layout-checklist',
    name: 'Checklist',
    category: 'Layout',
    description: 'Pit checklist. Items are separated by |; ticks are remembered on this dashboard and cleared with Reset.',
    slots: [],
    settings: [
      { key: 'items', label: 'Items (one per line)', kind: 'textarea', rows: 8, default: 'Bumpers on\nBattery above 12.5 V\nRadio linked\nMechanisms home\nPhotos taken' },
      { key: 'storageKey', label: 'Share ticks with (blank = this widget only)', kind: 'text', default: '', placeholder: 'e.g. pit-check' }
    ],
    defaultSize: { w: 4, h: 6 },
    minSize: { w: 3, h: 3 },
    component: ChecklistWidget
  });

  registerWidget({
    type: 'layout-stopwatch',
    name: 'Stopwatch',
    category: 'Layout',
    description: 'Start, stop and reset a stopwatch, or count down a set time (for example a 2:15 teleop clock). Runs on this screen only.',
    slots: [],
    settings: [
      {
        key: 'mode',
        label: 'Mode',
        kind: 'enum',
        default: 'stopwatch',
        options: [
          { value: 'stopwatch', label: 'Stopwatch' },
          { value: 'countdown', label: 'Countdown' }
        ]
      },
      { key: 'target', label: 'Countdown length', kind: 'number', default: 135, min: 1, max: 3600, step: 1, unit: 's' }
    ],
    defaultSize: { w: 4, h: 3 },
    minSize: { w: 3, h: 2 },
    component: StopwatchWidget
  });
}
