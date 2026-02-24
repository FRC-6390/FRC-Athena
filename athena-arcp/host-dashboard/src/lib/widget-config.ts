import type { SignalRow } from './arcp';
import { leafPath, type WidgetConfigRecord, type WidgetKind } from './dashboard';

const GRAPH_COLORS = ['#f43f5e', '#38bdf8', '#34d399', '#f59e0b', '#a78bfa', '#f97316', '#22d3ee'];
const DEFAULT_FIELD_LENGTH_METERS = 16.54;
const DEFAULT_FIELD_WIDTH_METERS = 8.02;
const HTML_INPUT_TYPES = [
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

const CONTROLLER_PARAM_ORDER = [
  'kp',
  'ki',
  'kd',
  'ks',
  'kv',
  'ka',
  'kg',
  'izone',
  'setpoint'
] as const;

const CONTROLLER_PARAM_ALIASES: Record<string, string[]> = {
  kp: ['kp', 'p', 'proportional'],
  ki: ['ki', 'i', 'integral'],
  kd: ['kd', 'd', 'derivative'],
  ks: ['ks', 's', 'static'],
  kv: ['kv', 'v', 'velocity'],
  ka: ['ka', 'a', 'accel', 'acceleration'],
  kg: ['kg', 'g', 'gravity'],
  izone: ['izone', 'integralzone', 'iz'],
  setpoint: ['setpoint', 'set', 'target', 'goal']
};

const CONTROLLER_PARAM_LABELS: Record<string, string> = {
  kp: 'kP',
  ki: 'kI',
  kd: 'kD',
  ks: 'kS',
  kv: 'kV',
  ka: 'kA',
  kg: 'kG',
  izone: 'I Zone',
  setpoint: 'Setpoint'
};

export type GraphSeriesStyle = 'line' | 'step' | 'dot';

export type GraphSeries = {
  signalId: number;
  label: string;
  color: string;
  style: GraphSeriesStyle;
  role: string;
};

export type GraphWidgetConfig = {
  series: GraphSeries[];
  yMin: number | null;
  yMax: number | null;
  showLegend: boolean;
};

export type ControllerParam = {
  key: string;
  label: string;
  signalId: number;
};

export type ControllerWidgetConfig = {
  params: ControllerParam[];
};

export type BarWidgetMode = 'progress' | 'usage';

export type BarWidgetConfig = {
  min: number;
  max: number;
  unit: string;
  mode: BarWidgetMode;
  warn: number;
  crit: number;
};

export type DialWidgetConfig = {
  min: number;
  max: number;
  unit: string;
};

export type TimerWidgetMode = 'elapsed' | 'countdown';

export type TimerWidgetConfig = {
  mode: TimerWidgetMode;
  durationSec: number;
};

export type DropdownOption = {
  label: string;
  value: string;
};

export type DropdownWidgetCommit = 'auto' | 'button';

export type DropdownWidgetConfig = {
  options: DropdownOption[];
  commit: DropdownWidgetCommit;
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

export type FieldWidgetConfig = {
  xSignalId: number | null;
  ySignalId: number | null;
  headingSignalId: number | null;
  trajectorySignalId: number | null;
  imageSignalId: number | null;
  imageUrl: string;
  imageOpacity: number;
  fieldLength: number;
  fieldWidth: number;
  allowPoseSet: boolean;
};

export type Imu3dUnits = 'deg' | 'rad';
export type ImuViewMode = 'auto' | '1d' | '2d' | '3d';

export type Imu3dWidgetConfig = {
  rollSignalId: number | null;
  pitchSignalId: number | null;
  yawSignalId: number | null;
  units: Imu3dUnits;
};

export type MotorWidgetConfig = {
  showOtherFields: boolean;
  canIdSignalId: number | null;
  canbusSignalId: number | null;
  typeSignalId: number | null;
  connectedSignalId: number | null;
  stalledSignalId: number | null;
  neutralModeSignalId: number | null;
  currentLimitSignalId: number | null;
  invertedSignalId: number | null;
  brakeModeSignalId: number | null;
  outputSignalId: number | null;
  velocitySignalId: number | null;
  positionSignalId: number | null;
  currentSignalId: number | null;
  temperatureSignalId: number | null;
  voltageSignalId: number | null;
  commandSignalId: number | null;
};

export type EncoderWidgetConfig = {
  positionSignalId: number | null;
  velocitySignalId: number | null;
  absoluteSignalId: number | null;
  connectedSignalId: number | null;
  ratioSignalId: number | null;
  offsetSignalId: number | null;
  canIdSignalId: number | null;
  canbusSignalId: number | null;
  typeSignalId: number | null;
  invertedSignalId: number | null;
  supportsSimulationSignalId: number | null;
  rawAbsoluteSignalId: number | null;
  positionViewMode: 'continuous' | 'zero_to_one' | 'neg180_to_180' | 'zero_to_360' | 'distance';
  absoluteViewMode: 'continuous' | 'zero_to_one' | 'neg180_to_180' | 'zero_to_360' | 'distance';
  positionMin: number;
  positionMax: number;
  absoluteMin: number;
  absoluteMax: number;
  positionUnit: string;
  absoluteUnit: string;
};

export type ImuWidgetConfig = {
  rollSignalId: number | null;
  pitchSignalId: number | null;
  yawSignalId: number | null;
  headingSignalId: number | null;
  connectedSignalId: number | null;
  accelSignalId: number | null;
  gyroSignalId: number | null;
  magSignalId: number | null;
  canIdSignalId: number | null;
  canbusSignalId: number | null;
  typeSignalId: number | null;
  invertedSignalId: number | null;
  orientationViewMode: ImuViewMode;
  accelViewMode: ImuViewMode;
  gyroViewMode: ImuViewMode;
  magViewMode: ImuViewMode;
  units: Imu3dUnits;
};

export type LayoutTitleConfig = {
  text: string;
  color: string;
};

export type LayoutGridConfig = {
  columns: number;
  rows: number;
  autoSize: boolean;
};

export type Mech2dWidgetConfig = {
  showGrid: boolean;
  lineWidth: number;
};

export type Mech2dSegment = {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  color: string;
  width: number;
};

export type Mech2dJoint = {
  x: number;
  y: number;
  r: number;
  color: string;
};

export type Mech2dScene = {
  viewportWidth: number;
  viewportHeight: number;
  segments: Mech2dSegment[];
  joints: Mech2dJoint[];
};

export type SwerveModuleWidgetConfig = {
  angleSignalId: number | null;
  speedSignalId: number | null;
  label: string;
  maxSpeed: number;
};

export type SwerveDriveModuleBinding = {
  key: string;
  label: string;
  angleSignalId: number | null;
  speedSignalId: number | null;
};

export type SwerveDriveWidgetConfig = {
  modules: SwerveDriveModuleBinding[];
  headingSignalId: number | null;
  maxSpeed: number;
};

export type DifferentialDriveWidgetConfig = {
  leftSpeedSignalId: number | null;
  rightSpeedSignalId: number | null;
  headingSignalId: number | null;
  maxSpeed: number;
};

export type FieldPoint = {
  x: number;
  y: number;
};

export type StatusMatrixHealth = 'true' | 'false';

export type StatusMatrixItem = {
  signalId: number;
  label: string;
  group: string;
  healthyWhen: StatusMatrixHealth;
};

export type StatusMatrixConfig = {
  items: StatusMatrixItem[];
  columns: number;
  showSummary: boolean;
};

export type CameraOverlayConfig = {
  streamSignalId: number | null;
  streamUrl: string;
  poseXSignalId: number | null;
  poseYSignalId: number | null;
  headingSignalId: number | null;
  targetsSignalId: number | null;
  detectionsSignalId: number | null;
  showPose: boolean;
  showTargets: boolean;
  showDetections: boolean;
  mirrorX: boolean;
  sourceWidth: number;
  sourceHeight: number;
};

export type CameraOverlayBox = {
  x: number;
  y: number;
  w: number;
  h: number;
  label: string;
  score: number | null;
};

function isObject(value: unknown): value is Record<string, unknown> {
  return !!value && typeof value === 'object' && !Array.isArray(value);
}

function toNumber(value: unknown): number | null {
  if (typeof value === 'number') {
    return Number.isFinite(value) ? value : null;
  }

  if (typeof value === 'string') {
    const numeric = Number(value.trim());
    return Number.isFinite(numeric) ? numeric : null;
  }

  return null;
}

function toBoolean(value: unknown, fallback: boolean): boolean {
  if (typeof value === 'boolean') return value;
  return fallback;
}

function toString(value: unknown, fallback = ''): string {
  if (typeof value === 'string') return value;
  return fallback;
}

function looksLikeColor(value: string): boolean {
  const raw = value.trim();
  if (!raw) return false;
  if (/^#([0-9a-fA-F]{3}|[0-9a-fA-F]{6}|[0-9a-fA-F]{8})$/.test(raw)) return true;
  if (/^(rgb|rgba|hsl|hsla)\(/i.test(raw)) return true;
  if (/^[a-zA-Z]+$/.test(raw)) return true;
  return false;
}

function clamp01(value: number): number {
  return Math.max(0, Math.min(1, value));
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

function isHtmlInputType(value: string): value is HtmlInputType {
  return (HTML_INPUT_TYPES as readonly string[]).includes(value);
}

function normalizeSignalId(value: unknown): number | null {
  const parsed = toNumber(value);
  if (parsed === null || parsed <= 0) return null;
  return Math.floor(parsed);
}

function looksLikeUrl(value: string): boolean {
  const raw = value.trim().toLowerCase();
  if (!raw) return false;
  return (
    raw.startsWith('http://') ||
    raw.startsWith('https://') ||
    raw.startsWith('rtsp://') ||
    raw.startsWith('mjpeg://') ||
    raw.startsWith('data:image/') ||
    raw.startsWith('/')
  );
}

function normalizeToken(raw: string): string {
  return raw.toLowerCase().replace(/[^a-z0-9]/g, '');
}

function parentPath(path: string): string {
  const parts = path.split('/').filter(Boolean);
  if (parts.length <= 1) return '';
  return parts.slice(0, -1).join('/');
}

function isNumericSignal(signal: SignalRow): boolean {
  return signal.signal_type === 'f64' || signal.signal_type === 'i64';
}

function signalRole(signal: SignalRow): string {
  if (signal.access === 'invoke' || signal.kind === 'command') return 'action';
  if (signal.access === 'write') return 'tunable';
  if (
    signal.signal_type === 'bool' &&
    signal.policy === 'on_change' &&
    signal.durability !== 'volatile'
  ) {
    return 'state';
  }
  return 'metric';
}

function graphColor(index: number): string {
  return GRAPH_COLORS[index % GRAPH_COLORS.length] ?? GRAPH_COLORS[0];
}

function graphSeriesFromSignal(signal: SignalRow, index: number): GraphSeries {
  return {
    signalId: signal.signal_id,
    label: leafPath(signal.path),
    color: graphColor(index),
    style: 'line',
    role: signalRole(signal)
  };
}

function parseGraphSeries(value: unknown, index: number): GraphSeries | null {
  if (!isObject(value)) return null;
  const signalId = toNumber(value.signalId);
  if (signalId === null || signalId <= 0) return null;

  const styleRaw = toString(value.style, 'line').toLowerCase();
  const style: GraphSeriesStyle =
    styleRaw === 'step' || styleRaw === 'dot' ? (styleRaw as GraphSeriesStyle) : 'line';

  const color = toString(value.color, graphColor(index)).trim() || graphColor(index);
  const label = toString(value.label, `signal ${signalId}`).trim() || `signal ${signalId}`;
  const role = toString(value.role, 'metric');

  return {
    signalId,
    style,
    color,
    label,
    role
  };
}

export function readGraphConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow
): GraphWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const items = Array.isArray(base.series)
    ? base.series
        .map((entry, index) => parseGraphSeries(entry, index))
        .filter((entry): entry is GraphSeries => entry !== null)
    : [];

  return {
    series: items.length > 0 ? items : [graphSeriesFromSignal(fallbackSignal, 0)],
    yMin: toNumber(base.yMin),
    yMax: toNumber(base.yMax),
    showLegend: toBoolean(base.showLegend, true)
  };
}

export function addGraphSeries(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  newSignal: SignalRow
): GraphWidgetConfig {
  const config = readGraphConfig(raw, fallbackSignal);
  if (config.series.some((series) => series.signalId === newSignal.signal_id)) {
    return config;
  }

  return {
    ...config,
    series: [...config.series, graphSeriesFromSignal(newSignal, config.series.length)]
  };
}

function guessControllerParamKey(signal: SignalRow): string | null {
  const leaf = normalizeToken(leafPath(signal.path));
  if (!leaf) return null;

  for (const [key, aliases] of Object.entries(CONTROLLER_PARAM_ALIASES)) {
    const normalizedAliases = aliases.map((alias) => normalizeToken(alias));
    if (normalizedAliases.some((alias) => alias === leaf || leaf.endsWith(alias))) {
      return key;
    }
  }

  return null;
}

function buildControllerDefaultParams(anchor: SignalRow, signals: SignalRow[]): ControllerParam[] {
  const anchorParent = parentPath(anchor.path);

  const siblings = signals.filter((signal) => {
    if (!isNumericSignal(signal)) return false;
    if (signal.access !== 'write') return false;
    if (signal.signal_id === anchor.signal_id) return true;

    const parent = parentPath(signal.path);
    if (!anchorParent) return parent === '';
    return parent === anchorParent;
  });

  const withKeys = siblings
    .map((signal) => ({
      signal,
      key: guessControllerParamKey(signal)
    }))
    .filter((entry) => entry.key !== null) as Array<{ signal: SignalRow; key: string }>;

  const uniqueByKey = new Map<string, SignalRow>();
  for (const entry of withKeys) {
    if (!uniqueByKey.has(entry.key)) {
      uniqueByKey.set(entry.key, entry.signal);
    }
  }

  const ordered: ControllerParam[] = [];
  for (const key of CONTROLLER_PARAM_ORDER) {
    const signal = uniqueByKey.get(key);
    if (!signal) continue;
    ordered.push({
      key,
      signalId: signal.signal_id,
      label: CONTROLLER_PARAM_LABELS[key] ?? key
    });
  }

  if (ordered.length > 0) {
    return ordered;
  }

  return [
    {
      key: 'value',
      signalId: anchor.signal_id,
      label: leafPath(anchor.path)
    }
  ];
}

export function readControllerConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): ControllerWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const parsed = Array.isArray(base.params)
    ? base.params
        .map((entry) => {
          if (!isObject(entry)) return null;
          const signalId = toNumber(entry.signalId);
          if (signalId === null || signalId <= 0) return null;
          const label = toString(entry.label, `Signal ${signalId}`).trim() || `Signal ${signalId}`;
          const key = toString(entry.key, normalizeToken(label) || 'value');
          return {
            key,
            label,
            signalId
          };
        })
        .filter((entry): entry is ControllerParam => entry !== null)
    : [];

  return {
    params: parsed.length > 0 ? parsed : buildControllerDefaultParams(fallbackSignal, signals)
  };
}

export function readBarConfig(raw: WidgetConfigRecord | undefined): BarWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const min = toNumber(base.min) ?? 0;
  const maxRaw = toNumber(base.max) ?? 1;
  const max = maxRaw <= min ? min + 1 : maxRaw;
  const modeRaw = toString(base.mode, 'progress').toLowerCase();
  const mode: BarWidgetMode = modeRaw === 'usage' ? 'usage' : 'progress';

  const warnRaw = toNumber(base.warn) ?? 0.7;
  const critRaw = toNumber(base.crit) ?? 0.9;
  const warn = clamp01(warnRaw);
  const crit = clamp01(Math.max(warn, critRaw));

  return {
    min,
    max,
    mode,
    warn,
    crit,
    unit: toString(base.unit, '')
  };
}

export function readDialConfig(raw: WidgetConfigRecord | undefined): DialWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const min = toNumber(base.min) ?? 0;
  const maxRaw = toNumber(base.max) ?? 100;
  return {
    min,
    max: maxRaw <= min ? min + 1 : maxRaw,
    unit: toString(base.unit, '')
  };
}

export function readTimerConfig(raw: WidgetConfigRecord | undefined): TimerWidgetConfig {
  const base = isObject(raw) ? raw : {};
  const modeRaw = toString(base.mode, 'countdown').toLowerCase();
  const mode: TimerWidgetMode = modeRaw === 'elapsed' ? 'elapsed' : 'countdown';
  return {
    mode,
    durationSec: Math.max(1, toNumber(base.durationSec) ?? 150)
  };
}

function defaultInputTypeForSignal(signal: SignalRow): HtmlInputType {
  if (signal.signal_type === 'f64' || signal.signal_type === 'i64') return 'number';
  return 'text';
}

function defaultInputCommitForType(inputType: HtmlInputType): InputWidgetCommit {
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

function defaultDropdownOptions(signal: SignalRow): DropdownOption[] {
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
    commit
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

function defaultStatusMatrixItems(anchor: SignalRow, signals: SignalRow[]): StatusMatrixItem[] {
  const anchorParent = parentPath(anchor.path);
  const siblings = signals
    .filter((signal) => {
      if (signal.signal_type !== 'bool') return false;
      if (signal.signal_id === anchor.signal_id) return true;
      return parentPath(signal.path) === anchorParent;
    })
    .sort((a, b) => a.path.localeCompare(b.path))
    .slice(0, 96);

  if (siblings.length === 0 && anchor.signal_type === 'bool') {
    siblings.push(anchor);
  }

  const fallbackGroup = anchorParent || 'status';
  return siblings.map((signal) => ({
    signalId: signal.signal_id,
    label: leafPath(signal.path),
    group: fallbackGroup,
    healthyWhen: 'true'
  }));
}

function parseStatusMatrixItem(value: unknown): StatusMatrixItem | null {
  if (!isObject(value)) return null;
  const signalId = normalizeSignalId(value.signalId);
  if (signalId === null) return null;

  const healthyWhenRaw = toString(value.healthyWhen, 'true').toLowerCase();
  const healthyWhen: StatusMatrixHealth = healthyWhenRaw === 'false' ? 'false' : 'true';

  return {
    signalId,
    label: toString(value.label, `Signal ${signalId}`).trim() || `Signal ${signalId}`,
    group: toString(value.group, 'status').trim() || 'status',
    healthyWhen
  };
}

export function readStatusMatrixConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): StatusMatrixConfig {
  const base = isObject(raw) ? raw : {};
  const items = Array.isArray(base.items)
    ? base.items
        .map((entry) => parseStatusMatrixItem(entry))
        .filter((entry): entry is StatusMatrixItem => entry !== null)
    : [];

  return {
    items: items.length > 0 ? items : defaultStatusMatrixItems(fallbackSignal, signals),
    columns: Math.max(1, Math.min(8, Math.floor(toNumber(base.columns) ?? 4))),
    showSummary: toBoolean(base.showSummary, true)
  };
}

export function addStatusMatrixSignal(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[],
  signal: SignalRow
): StatusMatrixConfig {
  const config = readStatusMatrixConfig(raw, fallbackSignal, signals);
  if (config.items.some((item) => item.signalId === signal.signal_id)) {
    return config;
  }

  return {
    ...config,
    items: [
      ...config.items,
      {
        signalId: signal.signal_id,
        label: leafPath(signal.path),
        group: parentPath(signal.path) || 'status',
        healthyWhen: 'true'
      }
    ]
  };
}

function defaultCameraOverlayConfig(anchor: SignalRow, signals: SignalRow[]): CameraOverlayConfig {
  const anchorParent = parentPath(anchor.path);
  const streamSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['stream', 'streamurl', 'url', 'camera', 'video', 'mjpeg'],
    ['string', 'string[]']
  );
  const targetsSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['targets', 'targetboxes', 'target', 'boxes'],
    ['string', 'string[]']
  );
  const detectionsSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['detections', 'objects', 'visionobjects', 'visionboxes'],
    ['string', 'string[]']
  );
  const poseX = findSignalByLeaf(signals, anchorParent, ['x', 'posex', 'robotx'], ['f64', 'i64']);
  const poseY = findSignalByLeaf(signals, anchorParent, ['y', 'posey', 'roboty'], ['f64', 'i64']);
  const heading = findSignalByLeaf(
    signals,
    anchorParent,
    ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
    ['f64', 'i64']
  );

  const streamFromAnchor =
    (anchor.signal_type === 'string' || anchor.signal_type === 'string[]') && looksLikeUrl(anchor.value)
      ? anchor.value.trim()
      : '';

  return {
    streamSignalId:
      streamSignal?.signal_id ??
      ((anchor.signal_type === 'string' || anchor.signal_type === 'string[]') ? anchor.signal_id : null),
    streamUrl: streamFromAnchor,
    poseXSignalId: poseX?.signal_id ?? null,
    poseYSignalId: poseY?.signal_id ?? null,
    headingSignalId: heading?.signal_id ?? null,
    targetsSignalId: targetsSignal?.signal_id ?? null,
    detectionsSignalId: detectionsSignal?.signal_id ?? null,
    showPose: true,
    showTargets: true,
    showDetections: true,
    mirrorX: false,
    sourceWidth: 1280,
    sourceHeight: 720
  };
}

export function readCameraOverlayConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): CameraOverlayConfig {
  const defaults = defaultCameraOverlayConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  const streamUrlRaw = toString(base.streamUrl, defaults.streamUrl).trim();
  const streamUrl = streamUrlRaw.length > 0 ? streamUrlRaw : defaults.streamUrl;

  return {
    streamSignalId: normalizeSignalId(base.streamSignalId) ?? defaults.streamSignalId,
    streamUrl,
    poseXSignalId: normalizeSignalId(base.poseXSignalId) ?? defaults.poseXSignalId,
    poseYSignalId: normalizeSignalId(base.poseYSignalId) ?? defaults.poseYSignalId,
    headingSignalId: normalizeSignalId(base.headingSignalId) ?? defaults.headingSignalId,
    targetsSignalId: normalizeSignalId(base.targetsSignalId) ?? defaults.targetsSignalId,
    detectionsSignalId: normalizeSignalId(base.detectionsSignalId) ?? defaults.detectionsSignalId,
    showPose: toBoolean(base.showPose, defaults.showPose),
    showTargets: toBoolean(base.showTargets, defaults.showTargets),
    showDetections: toBoolean(base.showDetections, defaults.showDetections),
    mirrorX: toBoolean(base.mirrorX, defaults.mirrorX),
    sourceWidth: Math.max(1, Math.floor(toNumber(base.sourceWidth) ?? defaults.sourceWidth)),
    sourceHeight: Math.max(1, Math.floor(toNumber(base.sourceHeight) ?? defaults.sourceHeight))
  };
}

function normalizedBoxFromUnknown(
  value: unknown,
  sourceWidth: number,
  sourceHeight: number
): CameraOverlayBox | null {
  if (!isObject(value)) return null;

  let x = toNumber(value.x ?? value.left);
  let y = toNumber(value.y ?? value.top);
  let w = toNumber(value.w ?? value.width);
  let h = toNumber(value.h ?? value.height);

  const x2 = toNumber(value.x2 ?? value.right);
  const y2 = toNumber(value.y2 ?? value.bottom);
  const cx = toNumber(value.cx ?? value.centerX);
  const cy = toNumber(value.cy ?? value.centerY);

  if (w === null && x !== null && x2 !== null) {
    w = x2 - x;
  }
  if (h === null && y !== null && y2 !== null) {
    h = y2 - y;
  }

  if (x === null && cx !== null && w !== null) {
    x = cx - w / 2;
  }
  if (y === null && cy !== null && h !== null) {
    y = cy - h / 2;
  }

  if (x === null || y === null || w === null || h === null) return null;
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(w) || !Number.isFinite(h)) {
    return null;
  }
  if (w <= 0 || h <= 0) return null;

  const maxAbs = Math.max(Math.abs(x), Math.abs(y), Math.abs(w), Math.abs(h));
  const pixelSpace = maxAbs > 1.5;

  const nx = pixelSpace ? x / sourceWidth : x;
  const ny = pixelSpace ? y / sourceHeight : y;
  const nw = pixelSpace ? w / sourceWidth : w;
  const nh = pixelSpace ? h / sourceHeight : h;

  const clampedX = clamp(nx, 0, 1);
  const clampedY = clamp(ny, 0, 1);
  const clampedW = clamp(nw, 0.001, 1);
  const clampedH = clamp(nh, 0.001, 1);

  return {
    x: clampedX,
    y: clampedY,
    w: clampedW,
    h: clampedH,
    label: toString(value.label ?? value.class ?? value.name ?? value.id, '').trim(),
    score: toNumber(value.score ?? value.confidence)
  };
}

export function parseOverlayBoxes(
  raw: string,
  sourceWidth: number,
  sourceHeight: number
): CameraOverlayBox[] {
  const trimmed = raw.trim();
  if (!trimmed) return [];

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    const candidates: unknown[] = [];

    if (Array.isArray(parsed)) {
      candidates.push(...parsed);
    } else if (isObject(parsed)) {
      if (Array.isArray(parsed.boxes)) candidates.push(...parsed.boxes);
      if (Array.isArray(parsed.targets)) candidates.push(...parsed.targets);
      if (Array.isArray(parsed.detections)) candidates.push(...parsed.detections);
      if (candidates.length === 0) candidates.push(parsed);
    }

    return candidates
      .map((entry) => normalizedBoxFromUnknown(entry, sourceWidth, sourceHeight))
      .filter((entry): entry is CameraOverlayBox => entry !== null)
      .slice(0, 200);
  } catch {
    return [];
  }
}

function findSignalByLeaf(
  signals: SignalRow[],
  anchorParent: string,
  candidates: string[],
  allowedTypes?: string[]
): SignalRow | null {
  const normalizedCandidates = new Set(candidates.map((candidate) => normalizeToken(candidate)));
  const parentScoped = signals.filter((signal) => parentPath(signal.path) === anchorParent);
  const fullPool = parentScoped.length > 0 ? parentScoped : signals;

  for (const signal of fullPool) {
    if (allowedTypes && !allowedTypes.includes(signal.signal_type)) continue;
    const token = normalizeToken(leafPath(signal.path));
    if (!token) continue;
    if (!normalizedCandidates.has(token)) continue;
    return signal;
  }

  for (const signal of fullPool) {
    if (allowedTypes && !allowedTypes.includes(signal.signal_type)) continue;
    const token = normalizeToken(leafPath(signal.path));
    if (!token) continue;
    for (const candidate of normalizedCandidates) {
      if (token.endsWith(candidate)) {
        return signal;
      }
    }
  }

  return null;
}

function defaultFieldSignalBinding(anchor: SignalRow, signals: SignalRow[]): FieldWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const xSignal = findSignalByLeaf(signals, anchorParent, ['x', 'posex', 'robotx'], ['f64', 'i64']);
  const ySignal = findSignalByLeaf(signals, anchorParent, ['y', 'posey', 'roboty'], ['f64', 'i64']);
  const headingSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
    ['f64', 'i64']
  );
  const trajectorySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['trajectory', 'traj', 'path'],
    ['string', 'string[]']
  );
  const imageSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['fieldimage', 'imageurl', 'image', 'fieldmap'],
    ['string', 'string[]']
  );

  const leaf = normalizeToken(leafPath(anchor.path));
  const xSignalId = xSignal?.signal_id ?? (isNumericSignal(anchor) && (leaf === 'x' || leaf.endsWith('x')) ? anchor.signal_id : null);
  const ySignalId = ySignal?.signal_id ?? (isNumericSignal(anchor) && (leaf === 'y' || leaf.endsWith('y')) ? anchor.signal_id : null);
  const headingSignalId = headingSignal?.signal_id ?? (isNumericSignal(anchor) && ['heading', 'yaw', 'theta', 'rotation', 'gyro'].some((token) => leaf.endsWith(token)) ? anchor.signal_id : null);

  const xWritable = xSignalId !== null && signals.find((signal) => signal.signal_id === xSignalId)?.access === 'write';
  const yWritable = ySignalId !== null && signals.find((signal) => signal.signal_id === ySignalId)?.access === 'write';

  return {
    xSignalId,
    ySignalId,
    headingSignalId,
    trajectorySignalId: trajectorySignal?.signal_id ?? null,
    imageSignalId: imageSignal?.signal_id ?? null,
    imageUrl: '/fields/FieldImage2026.svg',
    imageOpacity: 0.78,
    fieldLength: DEFAULT_FIELD_LENGTH_METERS,
    fieldWidth: DEFAULT_FIELD_WIDTH_METERS,
    allowPoseSet: !!xWritable && !!yWritable
  };
}

export function readFieldConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): FieldWidgetConfig {
  const defaults = defaultFieldSignalBinding(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  const xSignalId = toNumber(base.xSignalId);
  const ySignalId = toNumber(base.ySignalId);
  const headingSignalId = toNumber(base.headingSignalId);
  const trajectorySignalId = toNumber(base.trajectorySignalId);
  const imageSignalId = toNumber(base.imageSignalId);

  return {
    xSignalId: xSignalId !== null && xSignalId > 0 ? xSignalId : defaults.xSignalId,
    ySignalId: ySignalId !== null && ySignalId > 0 ? ySignalId : defaults.ySignalId,
    headingSignalId: headingSignalId !== null && headingSignalId > 0 ? headingSignalId : defaults.headingSignalId,
    trajectorySignalId:
      trajectorySignalId !== null && trajectorySignalId > 0
        ? trajectorySignalId
        : defaults.trajectorySignalId,
    imageSignalId: imageSignalId !== null && imageSignalId > 0 ? imageSignalId : defaults.imageSignalId,
    imageUrl: toString(base.imageUrl, defaults.imageUrl),
    imageOpacity: clamp(toNumber(base.imageOpacity) ?? defaults.imageOpacity, 0, 1),
    fieldLength: Math.max(1, toNumber(base.fieldLength) ?? defaults.fieldLength),
    fieldWidth: Math.max(1, toNumber(base.fieldWidth) ?? defaults.fieldWidth),
    allowPoseSet: toBoolean(base.allowPoseSet, defaults.allowPoseSet)
  };
}

function defaultImu3dConfig(anchor: SignalRow, signals: SignalRow[]): Imu3dWidgetConfig {
  const anchorParent = parentPath(anchor.path);
  const rollSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['roll', 'phi', 'bank'],
    ['f64', 'i64']
  );
  const pitchSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['pitch', 'theta', 'attitude'],
    ['f64', 'i64']
  );
  const yawSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['yaw', 'heading', 'psi', 'gyro', 'angle', 'bearing'],
    ['f64', 'i64']
  );

  const leaf = normalizeToken(leafPath(anchor.path));
  const rollSignalId =
    rollSignal?.signal_id ??
    (isNumericSignal(anchor) && ['roll', 'phi', 'bank'].some((token) => leaf.endsWith(token))
      ? anchor.signal_id
      : null);
  const pitchSignalId =
    pitchSignal?.signal_id ??
    (isNumericSignal(anchor) && ['pitch', 'theta', 'attitude'].some((token) => leaf.endsWith(token))
      ? anchor.signal_id
      : null);
  const yawSignalId =
    yawSignal?.signal_id ??
    (isNumericSignal(anchor) && ['yaw', 'heading', 'psi', 'bearing'].some((token) => leaf.endsWith(token))
      ? anchor.signal_id
      : null);

  const units: Imu3dUnits = anchor.path.toLowerCase().includes('rad') ? 'rad' : 'deg';

  return {
    rollSignalId,
    pitchSignalId,
    yawSignalId,
    units
  };
}

export function readImu3dConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): Imu3dWidgetConfig {
  const defaults = defaultImu3dConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const unitsRaw = toString(base.units, defaults.units).toLowerCase();
  const units: Imu3dUnits = unitsRaw === 'rad' ? 'rad' : 'deg';

  return {
    rollSignalId: normalizeSignalId(base.rollSignalId) ?? defaults.rollSignalId,
    pitchSignalId: normalizeSignalId(base.pitchSignalId) ?? defaults.pitchSignalId,
    yawSignalId: normalizeSignalId(base.yawSignalId) ?? defaults.yawSignalId,
    units
  };
}

function findWritableNumericSignalByLeaf(
  signals: SignalRow[],
  anchorParent: string,
  candidates: string[]
): SignalRow | null {
  const scoped = signals.filter((signal) => parentPath(signal.path) === anchorParent);
  const pool = scoped.length > 0 ? scoped : signals;
  const normalizedCandidates = candidates.map((candidate) => normalizeToken(candidate));

  for (const signal of pool) {
    if (!isNumericSignal(signal) || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.includes(token)) {
      return signal;
    }
  }

  for (const signal of pool) {
    if (!isNumericSignal(signal) || signal.access !== 'write') continue;
    const token = normalizeToken(leafPath(signal.path));
    if (normalizedCandidates.some((candidate) => token.endsWith(candidate) || token.includes(candidate))) {
      return signal;
    }
  }

  return null;
}

function defaultMotorConfig(anchor: SignalRow, signals: SignalRow[]): MotorWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const canIdSignal = findSignalByLeaf(signals, anchorParent, ['canid', 'can_id', 'id'], ['f64', 'i64']);
  const canbusSignal = findSignalByLeaf(signals, anchorParent, ['canbus', 'bus'], ['string']);
  const typeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['type', 'motortype', 'controller', 'controller_type'],
    ['string']
  );
  const connectedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['connected', 'is_connected', 'alive', 'online', 'faulted'],
    ['bool']
  );
  const stalledSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['stalled', 'stall', 'is_stalled'],
    ['bool']
  );
  const neutralModeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['neutral_mode', 'neutralmode', 'idle_mode', 'idlemode', 'mode'],
    ['string', 'bool']
  );
  const currentLimitSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['current_limit_a', 'currentlimita', 'current_limit', 'limit_amps', 'max_current'],
    ['f64', 'i64']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );
  const brakeModeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['brake_mode', 'brakemode'],
    ['bool', 'string']
  );
  const outputSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['output', 'percent_output', 'duty_cycle', 'applied_output', 'command', 'set'],
    ['f64', 'i64']
  );
  const velocitySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['velocity', 'speed', 'rpm', 'rate'],
    ['f64', 'i64']
  );
  const positionSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['position', 'rotations', 'distance', 'pos'],
    ['f64', 'i64']
  );
  const currentSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['current', 'amps', 'supply_current', 'stator_current'],
    ['f64', 'i64']
  );
  const temperatureSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['temperature', 'temp', 'temp_c', 'motor_temp'],
    ['f64', 'i64']
  );
  const voltageSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['voltage', 'bus_voltage', 'output_voltage'],
    ['f64', 'i64']
  );
  const commandSignal = findWritableNumericSignalByLeaf(
    signals,
    anchorParent,
    ['output', 'set', 'command', 'speed', 'velocity', 'voltage']
  );

  const fallbackOutput =
    isNumericSignal(anchor) &&
    ['output', 'command', 'set', 'speed', 'velocity'].some((token) =>
      normalizeToken(leafPath(anchor.path)).includes(normalizeToken(token))
    )
      ? anchor.signal_id
      : null;

  return {
    showOtherFields: true,
    canIdSignalId: canIdSignal?.signal_id ?? null,
    canbusSignalId: canbusSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null,
    connectedSignalId: connectedSignal?.signal_id ?? null,
    stalledSignalId: stalledSignal?.signal_id ?? null,
    neutralModeSignalId: neutralModeSignal?.signal_id ?? brakeModeSignal?.signal_id ?? null,
    currentLimitSignalId: currentLimitSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    brakeModeSignalId: brakeModeSignal?.signal_id ?? null,
    outputSignalId: outputSignal?.signal_id ?? fallbackOutput,
    velocitySignalId: velocitySignal?.signal_id ?? null,
    positionSignalId: positionSignal?.signal_id ?? null,
    currentSignalId: currentSignal?.signal_id ?? null,
    temperatureSignalId: temperatureSignal?.signal_id ?? null,
    voltageSignalId: voltageSignal?.signal_id ?? null,
    commandSignalId: commandSignal?.signal_id ?? null
  };
}

export function readMotorConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): MotorWidgetConfig {
  const defaults = defaultMotorConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    showOtherFields: toBoolean(base.showOtherFields, defaults.showOtherFields),
    canIdSignalId: normalizeSignalId(base.canIdSignalId) ?? defaults.canIdSignalId,
    canbusSignalId: normalizeSignalId(base.canbusSignalId) ?? defaults.canbusSignalId,
    typeSignalId: normalizeSignalId(base.typeSignalId) ?? defaults.typeSignalId,
    connectedSignalId: normalizeSignalId(base.connectedSignalId) ?? defaults.connectedSignalId,
    stalledSignalId: normalizeSignalId(base.stalledSignalId) ?? defaults.stalledSignalId,
    neutralModeSignalId:
      normalizeSignalId(base.neutralModeSignalId) ??
      normalizeSignalId(base.modeSignalId) ??
      defaults.neutralModeSignalId,
    currentLimitSignalId: normalizeSignalId(base.currentLimitSignalId) ?? defaults.currentLimitSignalId,
    invertedSignalId: normalizeSignalId(base.invertedSignalId) ?? defaults.invertedSignalId,
    brakeModeSignalId: normalizeSignalId(base.brakeModeSignalId) ?? defaults.brakeModeSignalId,
    outputSignalId: normalizeSignalId(base.outputSignalId) ?? defaults.outputSignalId,
    velocitySignalId: normalizeSignalId(base.velocitySignalId) ?? defaults.velocitySignalId,
    positionSignalId: normalizeSignalId(base.positionSignalId) ?? defaults.positionSignalId,
    currentSignalId: normalizeSignalId(base.currentSignalId) ?? defaults.currentSignalId,
    temperatureSignalId: normalizeSignalId(base.temperatureSignalId) ?? defaults.temperatureSignalId,
    voltageSignalId: normalizeSignalId(base.voltageSignalId) ?? defaults.voltageSignalId,
    commandSignalId: normalizeSignalId(base.commandSignalId) ?? defaults.commandSignalId
  };
}

function defaultEncoderConfig(anchor: SignalRow, signals: SignalRow[]): EncoderWidgetConfig {
  const anchorParent = parentPath(anchor.path);

  const positionSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['position', 'pos', 'rotations', 'distance'],
    ['f64', 'i64']
  );
  const velocitySignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['velocity', 'speed', 'rate', 'rpm'],
    ['f64', 'i64']
  );
  const absoluteSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['absolute', 'absolute_position', 'abs'],
    ['f64', 'i64']
  );
  const connectedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['connected', 'is_connected', 'healthy', 'online'],
    ['bool']
  );
  const ratioSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['ratio', 'gear_ratio', 'conversion'],
    ['f64', 'i64']
  );
  const offsetSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['offset', 'zero_offset', 'calibration_offset'],
    ['f64', 'i64']
  );
  const canIdSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['canid', 'can_id', 'id'],
    ['f64', 'i64']
  );
  const canbusSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['canbus', 'bus'],
    ['string']
  );
  const typeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['type', 'encoder_type', 'sensor_type'],
    ['string']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );
  const supportsSimulationSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['supports_simulation', 'sim_supported', 'simulation'],
    ['bool']
  );
  const rawAbsoluteSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['raw_absolute_rot', 'raw_absolute', 'raw'],
    ['f64', 'i64']
  );

  const fallbackPosition =
    isNumericSignal(anchor) &&
    ['position', 'rotations', 'distance'].some((token) =>
      normalizeToken(leafPath(anchor.path)).includes(normalizeToken(token))
    )
      ? anchor.signal_id
      : null;

  return {
    positionSignalId: positionSignal?.signal_id ?? fallbackPosition,
    velocitySignalId: velocitySignal?.signal_id ?? null,
    absoluteSignalId: absoluteSignal?.signal_id ?? null,
    connectedSignalId: connectedSignal?.signal_id ?? null,
    ratioSignalId: ratioSignal?.signal_id ?? null,
    offsetSignalId: offsetSignal?.signal_id ?? null,
    canIdSignalId: canIdSignal?.signal_id ?? null,
    canbusSignalId: canbusSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    supportsSimulationSignalId: supportsSimulationSignal?.signal_id ?? null,
    rawAbsoluteSignalId: rawAbsoluteSignal?.signal_id ?? null,
    positionViewMode: 'continuous',
    absoluteViewMode: 'zero_to_one',
    positionMin: -2.0,
    positionMax: 2.0,
    absoluteMin: 0.0,
    absoluteMax: 1.0,
    positionUnit: 'rot',
    absoluteUnit: 'rot'
  };
}

export function readEncoderConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): EncoderWidgetConfig {
  const defaults = defaultEncoderConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    positionSignalId: normalizeSignalId(base.positionSignalId) ?? defaults.positionSignalId,
    velocitySignalId: normalizeSignalId(base.velocitySignalId) ?? defaults.velocitySignalId,
    absoluteSignalId: normalizeSignalId(base.absoluteSignalId) ?? defaults.absoluteSignalId,
    connectedSignalId: normalizeSignalId(base.connectedSignalId) ?? defaults.connectedSignalId,
    ratioSignalId: normalizeSignalId(base.ratioSignalId) ?? defaults.ratioSignalId,
    offsetSignalId: normalizeSignalId(base.offsetSignalId) ?? defaults.offsetSignalId,
    canIdSignalId: normalizeSignalId(base.canIdSignalId) ?? defaults.canIdSignalId,
    canbusSignalId: normalizeSignalId(base.canbusSignalId) ?? defaults.canbusSignalId,
    typeSignalId: normalizeSignalId(base.typeSignalId) ?? defaults.typeSignalId,
    invertedSignalId: normalizeSignalId(base.invertedSignalId) ?? defaults.invertedSignalId,
    supportsSimulationSignalId:
      normalizeSignalId(base.supportsSimulationSignalId) ?? defaults.supportsSimulationSignalId,
    rawAbsoluteSignalId: normalizeSignalId(base.rawAbsoluteSignalId) ?? defaults.rawAbsoluteSignalId,
    positionViewMode:
      base.positionViewMode === 'zero_to_one' ||
      base.positionViewMode === 'continuous' ||
      base.positionViewMode === 'neg180_to_180' ||
      base.positionViewMode === 'zero_to_360' ||
      base.positionViewMode === 'distance'
        ? base.positionViewMode
        : defaults.positionViewMode,
    absoluteViewMode:
      base.absoluteViewMode === 'zero_to_one' ||
      base.absoluteViewMode === 'continuous' ||
      base.absoluteViewMode === 'neg180_to_180' ||
      base.absoluteViewMode === 'zero_to_360' ||
      base.absoluteViewMode === 'distance'
        ? base.absoluteViewMode
        : defaults.absoluteViewMode,
    positionMin: toNumber(base.positionMin) ?? defaults.positionMin,
    positionMax: toNumber(base.positionMax) ?? defaults.positionMax,
    absoluteMin: toNumber(base.absoluteMin) ?? defaults.absoluteMin,
    absoluteMax: toNumber(base.absoluteMax) ?? defaults.absoluteMax,
    positionUnit: toString(base.positionUnit, defaults.positionUnit),
    absoluteUnit: toString(base.absoluteUnit, defaults.absoluteUnit)
  };
}

function defaultImuConfig(anchor: SignalRow, signals: SignalRow[]): ImuWidgetConfig {
  const baseImu3d = defaultImu3dConfig(anchor, signals);
  const anchorParent = parentPath(anchor.path);

  const headingSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['heading', 'yaw', 'bearing', 'angle', 'gyro'],
    ['f64', 'i64']
  );
  const connectedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['connected', 'is_connected', 'healthy', 'online'],
    ['bool']
  );
  const accelSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['accel', 'acceleration', 'linear_accel'],
    ['f64[]', 'i64[]']
  );
  const gyroSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['gyro_rates', 'gyro', 'angular_velocity', 'rates'],
    ['f64[]', 'i64[]']
  );
  const magSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['mag_xyz_ut', 'mag_xyz', 'magnetometer', 'magnetic_field', 'mag'],
    ['f64[]', 'i64[]']
  );
  const canIdSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['canid', 'can_id', 'id'],
    ['f64', 'i64']
  );
  const canbusSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['canbus', 'bus'],
    ['string']
  );
  const typeSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['type', 'imu_type', 'sensor_type'],
    ['string']
  );
  const invertedSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['inverted', 'invert'],
    ['bool']
  );

  return {
    rollSignalId: baseImu3d.rollSignalId,
    pitchSignalId: baseImu3d.pitchSignalId,
    yawSignalId: baseImu3d.yawSignalId,
    headingSignalId: headingSignal?.signal_id ?? baseImu3d.yawSignalId,
    connectedSignalId: connectedSignal?.signal_id ?? null,
    accelSignalId: accelSignal?.signal_id ?? null,
    gyroSignalId: gyroSignal?.signal_id ?? null,
    magSignalId: magSignal?.signal_id ?? null,
    canIdSignalId: canIdSignal?.signal_id ?? null,
    canbusSignalId: canbusSignal?.signal_id ?? null,
    typeSignalId: typeSignal?.signal_id ?? null,
    invertedSignalId: invertedSignal?.signal_id ?? null,
    orientationViewMode: 'auto',
    accelViewMode: 'auto',
    gyroViewMode: 'auto',
    magViewMode: 'auto',
    units: baseImu3d.units
  };
}

export function readImuConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): ImuWidgetConfig {
  const defaults = defaultImuConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const unitsRaw = toString(base.units, defaults.units).toLowerCase();
  const units: Imu3dUnits = unitsRaw === 'rad' ? 'rad' : 'deg';

  return {
    rollSignalId: normalizeSignalId(base.rollSignalId) ?? defaults.rollSignalId,
    pitchSignalId: normalizeSignalId(base.pitchSignalId) ?? defaults.pitchSignalId,
    yawSignalId: normalizeSignalId(base.yawSignalId) ?? defaults.yawSignalId,
    headingSignalId: normalizeSignalId(base.headingSignalId) ?? defaults.headingSignalId,
    connectedSignalId: normalizeSignalId(base.connectedSignalId) ?? defaults.connectedSignalId,
    accelSignalId: normalizeSignalId(base.accelSignalId) ?? defaults.accelSignalId,
    gyroSignalId: normalizeSignalId(base.gyroSignalId) ?? defaults.gyroSignalId,
    magSignalId: normalizeSignalId(base.magSignalId) ?? defaults.magSignalId,
    canIdSignalId: normalizeSignalId(base.canIdSignalId) ?? defaults.canIdSignalId,
    canbusSignalId: normalizeSignalId(base.canbusSignalId) ?? defaults.canbusSignalId,
    typeSignalId: normalizeSignalId(base.typeSignalId) ?? defaults.typeSignalId,
    invertedSignalId: normalizeSignalId(base.invertedSignalId) ?? defaults.invertedSignalId,
    orientationViewMode:
      base.orientationViewMode === '1d' ||
      base.orientationViewMode === '2d' ||
      base.orientationViewMode === '3d' ||
      base.orientationViewMode === 'auto'
        ? base.orientationViewMode
        : defaults.orientationViewMode,
    accelViewMode:
      base.accelViewMode === '1d' ||
      base.accelViewMode === '2d' ||
      base.accelViewMode === '3d' ||
      base.accelViewMode === 'auto'
        ? base.accelViewMode
        : defaults.accelViewMode,
    gyroViewMode:
      base.gyroViewMode === '1d' ||
      base.gyroViewMode === '2d' ||
      base.gyroViewMode === '3d' ||
      base.gyroViewMode === 'auto'
        ? base.gyroViewMode
        : defaults.gyroViewMode,
    magViewMode:
      base.magViewMode === '1d' ||
      base.magViewMode === '2d' ||
      base.magViewMode === '3d' ||
      base.magViewMode === 'auto'
        ? base.magViewMode
        : defaults.magViewMode,
    units
  };
}

export function readLayoutTitleConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackText: string
): LayoutTitleConfig {
  const base = isObject(raw) ? raw : {};
  const text = toString(base.text, fallbackText).trim() || fallbackText;
  const colorRaw = toString(base.color, '#f5f7fb').trim();
  const color = looksLikeColor(colorRaw) ? colorRaw : '#f5f7fb';
  return { text, color };
}

export function readLayoutGridConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackColumns: number,
  fallbackRows: number
): LayoutGridConfig {
  const base = isObject(raw) ? raw : {};
  const columns = Math.max(1, Math.min(96, Math.floor(toNumber(base.columns) ?? fallbackColumns)));
  const rows = Math.max(1, Math.min(96, Math.floor(toNumber(base.rows) ?? fallbackRows)));
  const autoSize = toBoolean(base.autoSize, false);
  return { columns, rows, autoSize };
}

export function parseNumericArray(raw: string): number[] {
  const trimmed = raw.trim();
  if (!trimmed) return [];

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    const arrayLike = Array.isArray(parsed)
      ? parsed
      : isObject(parsed) && Array.isArray(parsed.values)
        ? parsed.values
        : null;
    if (arrayLike) {
      return arrayLike
        .map((entry) => toNumber(entry))
        .filter((entry): entry is number => entry !== null)
        .slice(0, 4096);
    }
  } catch {
    // fall through to token extraction
  }

  const matches = trimmed.match(/[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?/g);
  if (!matches) return [];
  return matches
    .map((entry) => Number(entry))
    .filter((entry) => Number.isFinite(entry))
    .slice(0, 4096);
}

const SWERVE_MODULE_KEYS = ['fl', 'fr', 'bl', 'br'] as const;
const SWERVE_ANGLE_ALIASES = ['angle', 'angledeg', 'heading', 'yaw', 'theta', 'azimuth', 'steer'];
const SWERVE_SPEED_ALIASES = ['speed', 'speedmps', 'velocity', 'vel', 'drive', 'drivespeed'];

function moduleTokenFromPath(path: string): string | null {
  const parts = path.toLowerCase().split('/').filter(Boolean);
  for (const part of parts) {
    if ((SWERVE_MODULE_KEYS as readonly string[]).includes(part)) {
      return part;
    }
  }
  return null;
}

function hasAlias(token: string, aliases: readonly string[]): boolean {
  return aliases.some((alias) => token === alias || token.endsWith(alias));
}

function findSwerveSignal(
  signals: SignalRow[],
  moduleKey: string,
  aliases: readonly string[]
): SignalRow | null {
  const modulePathToken = `/swerve/${moduleKey}/`;
  const inModule = signals.filter((signal) => {
    if (!isNumericSignal(signal)) return false;
    return signal.path.toLowerCase().includes(modulePathToken);
  });

  for (const signal of inModule) {
    const leaf = normalizeToken(leafPath(signal.path));
    if (hasAlias(leaf, aliases)) {
      return signal;
    }
  }

  for (const signal of inModule) {
    const leaf = normalizeToken(leafPath(signal.path));
    if (aliases.some((alias) => leaf.includes(alias))) {
      return signal;
    }
  }

  return null;
}

function defaultSwerveModuleConfig(
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveModuleWidgetConfig {
  const moduleKey = moduleTokenFromPath(fallbackSignal.path);
  let angleSignal = moduleKey ? findSwerveSignal(signals, moduleKey, SWERVE_ANGLE_ALIASES) : null;
  let speedSignal = moduleKey ? findSwerveSignal(signals, moduleKey, SWERVE_SPEED_ALIASES) : null;

  if (!angleSignal || !speedSignal) {
    const parent = parentPath(fallbackSignal.path);
    angleSignal =
      angleSignal ??
      findSignalByLeaf(signals, parent, SWERVE_ANGLE_ALIASES, ['f64', 'i64']);
    speedSignal =
      speedSignal ??
      findSignalByLeaf(signals, parent, SWERVE_SPEED_ALIASES, ['f64', 'i64']);
  }

  if (isNumericSignal(fallbackSignal)) {
    const leaf = normalizeToken(leafPath(fallbackSignal.path));
    if (!angleSignal && hasAlias(leaf, SWERVE_ANGLE_ALIASES)) {
      angleSignal = fallbackSignal;
    }
    if (!speedSignal && hasAlias(leaf, SWERVE_SPEED_ALIASES)) {
      speedSignal = fallbackSignal;
    }
  }

  const fallbackLabel = moduleKey
    ? moduleKey.toUpperCase()
    : leafPath(parentPath(fallbackSignal.path) || fallbackSignal.path).toUpperCase();

  return {
    angleSignalId: angleSignal?.signal_id ?? null,
    speedSignalId: speedSignal?.signal_id ?? null,
    label: fallbackLabel,
    maxSpeed: 5
  };
}

export function readSwerveModuleConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveModuleWidgetConfig {
  const defaults = defaultSwerveModuleConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    angleSignalId: normalizeSignalId(base.angleSignalId) ?? defaults.angleSignalId,
    speedSignalId: normalizeSignalId(base.speedSignalId) ?? defaults.speedSignalId,
    label: toString(base.label, defaults.label).trim() || defaults.label,
    maxSpeed: Math.max(0.1, toNumber(base.maxSpeed) ?? defaults.maxSpeed)
  };
}

function defaultSwerveDriveConfig(
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveDriveWidgetConfig {
  const modules = SWERVE_MODULE_KEYS.map((key) => {
    const angleSignal = findSwerveSignal(signals, key, SWERVE_ANGLE_ALIASES);
    const speedSignal = findSwerveSignal(signals, key, SWERVE_SPEED_ALIASES);
    return {
      key,
      label: key.toUpperCase(),
      angleSignalId: angleSignal?.signal_id ?? null,
      speedSignalId: speedSignal?.signal_id ?? null
    };
  });

  const headingSignal =
    findSignalByLeaf(
      signals,
      parentPath(fallbackSignal.path),
      ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/swerve/') && path.endsWith('/heading_deg');
    }) ??
    null;

  return {
    modules,
    headingSignalId: headingSignal?.signal_id ?? null,
    maxSpeed: 5
  };
}

export function readSwerveDriveConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): SwerveDriveWidgetConfig {
  const defaults = defaultSwerveDriveConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  const modules = [...defaults.modules];
  if (Array.isArray(base.modules)) {
    for (const entry of base.modules) {
      if (!isObject(entry)) continue;
      const key = toString(entry.key, '').toLowerCase();
      const idx = modules.findIndex((module) => module.key === key);
      if (idx < 0) continue;
      modules[idx] = {
        ...modules[idx],
        label: toString(entry.label, modules[idx].label).trim() || modules[idx].label,
        angleSignalId: normalizeSignalId(entry.angleSignalId) ?? modules[idx].angleSignalId,
        speedSignalId: normalizeSignalId(entry.speedSignalId) ?? modules[idx].speedSignalId
      };
    }
  }

  return {
    modules,
    headingSignalId: normalizeSignalId(base.headingSignalId) ?? defaults.headingSignalId,
    maxSpeed: Math.max(0.1, toNumber(base.maxSpeed) ?? defaults.maxSpeed)
  };
}

function defaultDifferentialDriveConfig(
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): DifferentialDriveWidgetConfig {
  const parent = parentPath(fallbackSignal.path);
  const left =
    findSignalByLeaf(
      signals,
      parent,
      ['leftspeed', 'left_speed', 'leftvelocity', 'left_vel', 'left'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/diff/') && path.includes('left') && path.includes('speed');
    }) ??
    null;
  const right =
    findSignalByLeaf(
      signals,
      parent,
      ['rightspeed', 'right_speed', 'rightvelocity', 'right_vel', 'right'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/diff/') && path.includes('right') && path.includes('speed');
    }) ??
    null;
  const heading =
    findSignalByLeaf(
      signals,
      parent,
      ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
      ['f64', 'i64']
    ) ??
    signals.find((signal) => {
      if (!isNumericSignal(signal)) return false;
      const path = signal.path.toLowerCase();
      return path.includes('/diff/') && path.includes('heading');
    }) ??
    null;

  return {
    leftSpeedSignalId: left?.signal_id ?? null,
    rightSpeedSignalId: right?.signal_id ?? null,
    headingSignalId: heading?.signal_id ?? null,
    maxSpeed: 5
  };
}

export function readDifferentialDriveConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): DifferentialDriveWidgetConfig {
  const defaults = defaultDifferentialDriveConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    leftSpeedSignalId: normalizeSignalId(base.leftSpeedSignalId) ?? defaults.leftSpeedSignalId,
    rightSpeedSignalId: normalizeSignalId(base.rightSpeedSignalId) ?? defaults.rightSpeedSignalId,
    headingSignalId: normalizeSignalId(base.headingSignalId) ?? defaults.headingSignalId,
    maxSpeed: Math.max(0.1, toNumber(base.maxSpeed) ?? defaults.maxSpeed)
  };
}

export function readMech2dConfig(raw: WidgetConfigRecord | undefined): Mech2dWidgetConfig {
  const base = isObject(raw) ? raw : {};
  return {
    showGrid: toBoolean(base.showGrid, true),
    lineWidth: clamp(toNumber(base.lineWidth) ?? 0.024, 0.002, 0.25)
  };
}

function parseMech2dSegment(value: unknown): Mech2dSegment | null {
  if (!isObject(value)) return null;
  const x1 = toNumber(value.x1);
  const y1 = toNumber(value.y1);
  const x2 = toNumber(value.x2);
  const y2 = toNumber(value.y2);
  if (x1 === null || y1 === null || x2 === null || y2 === null) return null;
  return {
    x1,
    y1,
    x2,
    y2,
    color: toString(value.color, '#f87171'),
    width: clamp(toNumber(value.width) ?? 0.024, 0.002, 0.25)
  };
}

function parseMech2dJoint(value: unknown): Mech2dJoint | null {
  if (!isObject(value)) return null;
  const x = toNumber(value.x);
  const y = toNumber(value.y);
  if (x === null || y === null) return null;
  return {
    x,
    y,
    r: clamp(toNumber(value.r) ?? 0.018, 0.002, 0.3),
    color: toString(value.color, '#dbeafe')
  };
}

export function parseMech2dScene(raw: string): Mech2dScene {
  const fallback: Mech2dScene = {
    viewportWidth: 1,
    viewportHeight: 1,
    segments: [],
    joints: []
  };
  const trimmed = raw.trim();
  if (!trimmed) return fallback;

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    if (!isObject(parsed)) return fallback;

    const viewportArray = Array.isArray(parsed.viewport) ? parsed.viewport : null;
    const viewportWidth = Math.max(
      0.1,
      toNumber(viewportArray?.[0] ?? parsed.viewportWidth ?? parsed.width) ?? fallback.viewportWidth
    );
    const viewportHeight = Math.max(
      0.1,
      toNumber(viewportArray?.[1] ?? parsed.viewportHeight ?? parsed.height) ?? fallback.viewportHeight
    );

    const segments = Array.isArray(parsed.segments)
      ? parsed.segments
          .map((entry) => parseMech2dSegment(entry))
          .filter((entry): entry is Mech2dSegment => entry !== null)
          .slice(0, 512)
      : [];

    const joints = Array.isArray(parsed.joints)
      ? parsed.joints
          .map((entry) => parseMech2dJoint(entry))
          .filter((entry): entry is Mech2dJoint => entry !== null)
          .slice(0, 512)
      : [];

    return {
      viewportWidth,
      viewportHeight,
      segments,
      joints
    };
  } catch {
    return fallback;
  }
}

function pointFromUnknown(value: unknown): FieldPoint | null {
  if (Array.isArray(value) && value.length >= 2) {
    const x = toNumber(value[0]);
    const y = toNumber(value[1]);
    if (x !== null && y !== null) {
      return { x, y };
    }
    return null;
  }

  if (!isObject(value)) return null;
  const x = toNumber(value.x);
  const y = toNumber(value.y);
  if (x !== null && y !== null) {
    return { x, y };
  }

  return null;
}

export function parseTrajectoryPoints(raw: string): FieldPoint[] {
  const trimmed = raw.trim();
  if (!trimmed) return [];

  try {
    const parsed = JSON.parse(trimmed) as unknown;

    const candidates = Array.isArray(parsed)
      ? parsed
      : isObject(parsed) && Array.isArray(parsed.points)
        ? parsed.points
        : [];

    return candidates
      .map((entry) => pointFromUnknown(entry))
      .filter((entry): entry is FieldPoint => entry !== null)
      .slice(0, 1000);
  } catch {
    // Fallback: parse "x,y;x,y" representation.
  }

  const points = trimmed
    .split(';')
    .map((pair) => pair.trim())
    .filter((pair) => pair.length > 0)
    .map((pair) => {
      const [xRaw, yRaw] = pair.split(',');
      const x = toNumber(xRaw);
      const y = toNumber(yRaw);
      if (x === null || y === null) return null;
      return { x, y };
    })
    .filter((entry): entry is FieldPoint => entry !== null);

  return points.slice(0, 1000);
}

export function cloneWidgetConfig(
  config: WidgetConfigRecord | undefined
): WidgetConfigRecord | undefined {
  if (!isObject(config)) return undefined;

  if (typeof globalThis.structuredClone === 'function') {
    try {
      return globalThis.structuredClone(config) as WidgetConfigRecord;
    } catch {
      // fall through to JSON clone
    }
  }

  try {
    return JSON.parse(JSON.stringify(config)) as WidgetConfigRecord;
  } catch {
    return { ...config };
  }
}

export function buildDefaultWidgetConfig(
  kind: WidgetKind,
  signal: SignalRow,
  signals: SignalRow[]
): WidgetConfigRecord | undefined {
  switch (kind) {
    case 'motor':
      return readMotorConfig(undefined, signal, signals);
    case 'encoder':
      return readEncoderConfig(undefined, signal, signals);
    case 'imu':
      return readImuConfig(undefined, signal, signals);
    case 'layout_title':
      return readLayoutTitleConfig(undefined, 'Title');
    case 'graph':
      return readGraphConfig(undefined, signal);
    case 'controller':
      return readControllerConfig(undefined, signal, signals);
    case 'bar':
      return readBarConfig(undefined);
    case 'dial':
      return readDialConfig(undefined);
    case 'timer':
      return readTimerConfig(undefined);
    case 'toggle':
      return readToggleConfig(undefined);
    case 'button_group':
    case 'radio':
      return readChoiceConfig(undefined, signal);
    case 'input':
      return readInputConfig(undefined, signal);
    case 'textarea':
      return readTextAreaConfig(undefined);
    case 'dropdown':
      return readDropdownConfig(undefined, signal);
    case 'field':
      return readFieldConfig(undefined, signal, signals);
    case 'imu_3d':
      return readImu3dConfig(undefined, signal, signals);
    case 'mech2d':
      return readMech2dConfig(undefined);
    case 'swerve_module':
      return readSwerveModuleConfig(undefined, signal, signals);
    case 'swerve_drive':
      return readSwerveDriveConfig(undefined, signal, signals);
    case 'differential_drive':
      return readDifferentialDriveConfig(undefined, signal, signals);
    case 'status_matrix':
      return readStatusMatrixConfig(undefined, signal, signals);
    case 'camera_overlay':
      return readCameraOverlayConfig(undefined, signal, signals);
    default:
      return undefined;
  }
}
