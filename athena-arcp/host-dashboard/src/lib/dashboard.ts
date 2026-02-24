import type { SignalRow } from './arcp';

const LEGACY_WIDGET_STORAGE_KEY = 'arcp.host.widgets.v1';
const LAYOUT_STORAGE_KEY = 'arcp.host.layout.v2';
const LAYOUT_EXPORT_FORMAT = 'arcp.host.layout.export';
const LAYOUT_EXPORT_VERSION = 1;

export const DEFAULT_GRID_COLUMNS = 24;

export const ROLE_OPTIONS = ['all', 'metric', 'state', 'tunable', 'action'] as const;
export const TYPE_OPTIONS = [
  'all',
  'bool',
  'i64',
  'f64',
  'string',
  'bool[]',
  'i64[]',
  'f64[]',
  'string[]'
] as const;

export const LAYOUT_TOOL_KINDS = [
  'layout_list',
  'layout_grid',
  'layout_divider',
  'layout_spacer',
  'layout_section',
  'layout_title'
] as const;

export type LayoutToolKind = (typeof LAYOUT_TOOL_KINDS)[number];
export type SignalWidgetKind =
  | 'metric'
  | 'state'
  | 'text'
  | 'trend'
  | 'action'
  | 'tunable'
  | 'motor'
  | 'encoder'
  | 'imu'
  | 'toggle'
  | 'button_group'
  | 'radio'
  | 'input'
  | 'textarea'
  | 'graph'
  | 'controller'
  | 'bar'
  | 'dial'
  | 'compass'
  | 'timer'
  | 'dropdown'
  | 'field'
  | 'imu_3d'
  | 'mech2d'
  | 'swerve_module'
  | 'swerve_drive'
  | 'differential_drive'
  | 'status_matrix'
  | 'camera_overlay';
export type WidgetKind = SignalWidgetKind | LayoutToolKind;
export type WidgetConfigRecord = Record<string, unknown>;

export type WidgetLayout = {
  x: number;
  y: number;
  w: number;
  h: number;
};

export type DashboardWidget = {
  id: string;
  signalId: number;
  kind: WidgetKind;
  title: string;
  layout: WidgetLayout;
  parentLayoutId?: string;
  config?: WidgetConfigRecord;
};

export type DashboardTab = {
  id: string;
  name: string;
  widgets: DashboardWidget[];
};

export type DashboardLayoutState = {
  activeTabId: string;
  tabs: DashboardTab[];
};

export type DashboardLayoutExport = {
  format: string;
  version: number;
  exported_at: string;
  layout: DashboardLayoutState;
};

export function signalRole(signal: SignalRow): string {
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

export function isActionSignal(signal: SignalRow): boolean {
  return signal.access === 'invoke' || signalRole(signal) === 'action';
}

export function isWritableSignal(signal: SignalRow): boolean {
  return signal.access === 'write' || signalRole(signal) === 'tunable';
}

export function widgetKindsFor(signal: SignalRow | null): WidgetKind[] {
  if (!signal) {
    return [
      'metric',
      'state',
      'text',
      'trend',
      'motor',
      'encoder',
      'imu',
      'graph',
      'bar',
      'dial',
      'compass',
      'timer',
      'field',
      'imu_3d',
      'mech2d',
      'swerve_module',
      'swerve_drive',
      'differential_drive',
      'dropdown',
      'status_matrix',
      'input',
      'textarea',
      'radio',
      'button_group',
      'toggle'
    ];
  }
  const hardwareKinds: WidgetKind[] = [];
  if (looksLikeMotorPath(signal.path)) hardwareKinds.push('motor');
  if (looksLikeEncoderPath(signal.path)) hardwareKinds.push('encoder');
  if (looksLikeImuPath(signal.path)) hardwareKinds.push('imu');

  if (isActionSignal(signal)) return ['action', 'state', 'text'];
  if (signal.signal_type === 'bool') {
    if (isWritableSignal(signal)) {
      return [
        ...hardwareKinds,
        'toggle',
        'button_group',
        'radio',
        'dropdown',
        'tunable',
        'status_matrix',
        'state',
        'text',
        'metric'
      ];
    }
    return [...hardwareKinds, 'status_matrix', 'state', 'metric', 'text'];
  }
  if (signal.signal_type === 'string' || signal.signal_type === 'string[]') {
    if (isWritableSignal(signal)) {
      return [
        ...hardwareKinds,
        'input',
        'textarea',
        'button_group',
        'radio',
        'dropdown',
        'mech2d',
        'camera_overlay',
        'tunable',
        'text',
        'state'
      ];
    }
    return [...hardwareKinds, 'mech2d', 'camera_overlay', 'text', 'state'];
  }
  if (signal.signal_type === 'f64' || signal.signal_type === 'i64') {
    if (isWritableSignal(signal)) {
      return [
        ...hardwareKinds,
        'input',
        'tunable',
        'controller',
        'graph',
        'bar',
        'dial',
        'compass',
        'timer',
        'swerve_module',
        'swerve_drive',
        'differential_drive',
        'dropdown',
        'field',
        'imu_3d',
        'camera_overlay',
        'metric',
        'trend',
        'text'
      ];
    }
    return [
      ...hardwareKinds,
      'metric',
      'graph',
      'trend',
      'bar',
      'dial',
      'compass',
      'timer',
      'controller',
      'swerve_module',
      'swerve_drive',
      'differential_drive',
      'field',
      'imu_3d',
      'camera_overlay',
      'text'
    ];
  }
  if (signal.signal_type === 'f64[]' || signal.signal_type === 'i64[]') {
    return [...hardwareKinds, 'imu_3d', 'swerve_drive', 'swerve_module', 'differential_drive', 'field', 'text', 'state'];
  }
  if (isWritableSignal(signal)) {
    return [...hardwareKinds, 'tunable', 'dropdown', 'text', 'state'];
  }
  return [...hardwareKinds, 'text', 'state'];
}

export function isLayoutWidgetKind(kind: WidgetKind | string): kind is LayoutToolKind {
  return (LAYOUT_TOOL_KINDS as readonly string[]).includes(kind);
}

export function widgetKindLabel(kind: WidgetKind): string {
  switch (kind) {
    case 'metric':
      return 'Metric';
    case 'state':
      return 'State';
    case 'text':
      return 'Text';
    case 'trend':
      return 'Trend';
    case 'action':
      return 'Action';
    case 'tunable':
      return 'Tunable';
    case 'motor':
      return 'Motor';
    case 'encoder':
      return 'Encoder';
    case 'imu':
      return 'IMU';
    case 'toggle':
      return 'Toggle';
    case 'button_group':
      return 'Buttons';
    case 'radio':
      return 'Radio';
    case 'input':
      return 'Input';
    case 'textarea':
      return 'Text Area';
    case 'graph':
      return 'Graph';
    case 'controller':
      return 'PID/FF Editor';
    case 'bar':
      return 'Bar/Meter';
    case 'dial':
      return 'Dial';
    case 'compass':
      return 'Compass';
    case 'timer':
      return 'Timer';
    case 'dropdown':
      return 'Dropdown';
    case 'field':
      return 'Field Viewer';
    case 'imu_3d':
      return '3D IMU';
    case 'mech2d':
      return 'Mechanism2d';
    case 'swerve_module':
      return 'Swerve Module';
    case 'swerve_drive':
      return 'Swerve Drive';
    case 'differential_drive':
      return 'Differential Drive';
    case 'status_matrix':
      return 'Status Board';
    case 'camera_overlay':
      return 'Camera + Overlay';
    case 'layout_list':
      return 'List';
    case 'layout_grid':
      return 'Grid';
    case 'layout_divider':
      return 'Divider';
    case 'layout_spacer':
      return 'Spacer';
    case 'layout_section':
      return 'Section';
    case 'layout_title':
      return 'Title';
    default:
      return kind;
  }
}

export function defaultWidgetKind(signal: SignalRow): WidgetKind {
  if (isActionSignal(signal)) return 'action';
  if (isWritableSignal(signal)) return 'tunable';
  if (looksLikeMotorPath(signal.path)) return 'motor';
  if (looksLikeEncoderPath(signal.path)) return 'encoder';
  if (looksLikeImuPath(signal.path)) return 'imu';
  if (signal.signal_type === 'f64[]' || signal.signal_type === 'i64[]') {
    const leaf = leafPath(signal.path).toLowerCase();
    if (
      leaf.includes('imu') ||
      leaf.includes('rpy') ||
      leaf.includes('orientation') ||
      leaf.includes('quaternion') ||
      leaf.includes('euler')
    ) {
      return 'imu_3d';
    }
    return 'text';
  }
  if (signal.signal_type === 'f64' || signal.signal_type === 'i64') {
    const leaf = leafPath(signal.path).toLowerCase();
    if (
      leaf.includes('imu') ||
      leaf.includes('roll') ||
      leaf.includes('pitch') ||
      leaf.includes('quaternion') ||
      leaf.includes('euler')
    ) {
      return 'imu_3d';
    }
    if (
      leaf.includes('heading') ||
      leaf.includes('yaw') ||
      leaf.includes('theta') ||
      leaf.includes('gyro') ||
      leaf.includes('azimuth') ||
      leaf.includes('bearing')
    ) {
      return 'compass';
    }
    return 'metric';
  }
  if (signal.signal_type === 'bool') return 'state';
  return 'text';
}

export function defaultLayoutFor(kind: WidgetKind): WidgetLayout {
  switch (kind) {
    case 'motor':
      return { x: 1, y: 1, w: 4, h: 2 };
    case 'encoder':
      return { x: 1, y: 1, w: 4, h: 2 };
    case 'imu':
      return { x: 1, y: 1, w: 5, h: 3 };
    case 'layout_list':
      return { x: 1, y: 1, w: 3, h: 2 };
    case 'layout_grid':
      return { x: 1, y: 1, w: 4, h: 2 };
    case 'layout_divider':
      return { x: 1, y: 1, w: 4, h: 1 };
    case 'layout_spacer':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'layout_section':
      return { x: 1, y: 1, w: 4, h: 1 };
    case 'layout_title':
      return { x: 1, y: 1, w: 6, h: 2 };
    case 'graph':
      return { x: 1, y: 1, w: 6, h: 3 };
    case 'controller':
      return { x: 1, y: 1, w: 4, h: 3 };
    case 'bar':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'dial':
      return { x: 1, y: 1, w: 2, h: 2 };
    case 'compass':
      return { x: 1, y: 1, w: 2, h: 2 };
    case 'timer':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'toggle':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'button_group':
      return { x: 1, y: 1, w: 3, h: 2 };
    case 'radio':
      return { x: 1, y: 1, w: 3, h: 2 };
    case 'input':
      return { x: 1, y: 1, w: 3, h: 1 };
    case 'textarea':
      return { x: 1, y: 1, w: 4, h: 2 };
    case 'dropdown':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'field':
      return { x: 1, y: 1, w: 6, h: 4 };
    case 'imu_3d':
      return { x: 1, y: 1, w: 4, h: 3 };
    case 'mech2d':
      return { x: 1, y: 1, w: 4, h: 4 };
    case 'swerve_module':
      return { x: 1, y: 1, w: 3, h: 3 };
    case 'swerve_drive':
      return { x: 1, y: 1, w: 6, h: 4 };
    case 'differential_drive':
      return { x: 1, y: 1, w: 5, h: 3 };
    case 'status_matrix':
      return { x: 1, y: 1, w: 4, h: 3 };
    case 'camera_overlay':
      return { x: 1, y: 1, w: 6, h: 4 };
    case 'trend':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'text':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'tunable':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'metric':
      return { x: 1, y: 1, w: 2, h: 1 };
    case 'state':
    case 'action':
      return { x: 1, y: 1, w: 2, h: 1 };
    default:
      return { x: 1, y: 1, w: 2, h: 1 };
  }
}

function pathLooksLike(path: string, tokens: string[]): boolean {
  const normalized = path.toLowerCase();
  return tokens.some((token) => normalized.includes(token));
}

function looksLikeMotorPath(path: string): boolean {
  return pathLooksLike(path, ['/motor', '/motors/', '/drive/', '/steer/', '/falcon', '/spark']);
}

function looksLikeEncoderPath(path: string): boolean {
  return pathLooksLike(path, ['/encoder', '/encoders/', '/absoluteencoder', '/cancoder', '/dutycycle']);
}

function looksLikeImuPath(path: string): boolean {
  return pathLooksLike(path, ['/imu', '/gyro', '/pigeon', '/navx', '/orientation', '/attitude']);
}

export function defaultLayoutTitle(kind: LayoutToolKind): string {
  switch (kind) {
    case 'layout_list':
      return 'List Block';
    case 'layout_grid':
      return 'Grid Block';
    case 'layout_divider':
      return 'Divider';
    case 'layout_spacer':
      return 'Spacer';
    case 'layout_section':
      return 'Section Header';
    case 'layout_title':
      return 'Title';
    default:
      return 'Layout Block';
  }
}

export function clampLayout(layout: WidgetLayout, columns = DEFAULT_GRID_COLUMNS): WidgetLayout {
  const minSpan = 1e-3;
  const w = Math.max(minSpan, Math.min(columns, layout.w));
  const x = Math.max(1, Math.min(columns - w + 1, layout.x));
  return {
    x,
    y: Math.max(1, layout.y),
    w,
    h: Math.max(minSpan, layout.h)
  };
}

export function layoutsOverlap(a: WidgetLayout, b: WidgetLayout): boolean {
  const ax2 = a.x + a.w;
  const ay2 = a.y + a.h;
  const bx2 = b.x + b.w;
  const by2 = b.y + b.h;

  if (ax2 <= b.x) return false;
  if (bx2 <= a.x) return false;
  if (ay2 <= b.y) return false;
  if (by2 <= a.y) return false;
  return true;
}

function nextId(prefix: string): string {
  const rand = Math.random().toString(36).slice(2, 8);
  return `${prefix}-${Date.now()}-${rand}`;
}

export function leafPath(path: string): string {
  const parts = path.split('/').filter(Boolean);
  return parts[parts.length - 1] ?? path;
}

function parentPath(path: string): string {
  const parts = path.split('/').filter(Boolean);
  if (parts.length <= 1) return '';
  return `/${parts.slice(0, -1).join('/')}`;
}

function defaultSignalWidgetTitle(signal: SignalRow, kind: WidgetKind): string {
  if (kind === 'motor' || kind === 'encoder' || kind === 'imu') {
    const parentLeaf = leafPath(parentPath(signal.path));
    if (parentLeaf && parentLeaf !== signal.path) {
      return parentLeaf;
    }
    return widgetKindLabel(kind);
  }
  return leafPath(signal.path);
}

export function findNextWidgetPlacement(
  widgets: DashboardWidget[],
  targetLayout: WidgetLayout,
  columns = DEFAULT_GRID_COLUMNS
): WidgetLayout {
  const draft = clampLayout(targetLayout, columns);
  const maxColStart = columns - draft.w + 1;

  const scan = (rowStart: number, rowEnd: number, colStartForFirstRow = 1): WidgetLayout | null => {
    for (let row = rowStart; row <= rowEnd; row++) {
      const startCol = row === rowStart ? colStartForFirstRow : 1;
      for (let col = startCol; col <= maxColStart; col++) {
        const candidate = { ...draft, x: col, y: row };
        const collision = widgets.some((widget) => {
          if (isLayoutWidgetKind(widget.kind)) return false;
          return layoutsOverlap(widget.layout, candidate);
        });
        if (!collision) {
          return candidate;
        }
      }
    }
    return null;
  };

  const preferred = scan(draft.y, 300, draft.x);
  if (preferred) {
    return preferred;
  }

  const wraparound = scan(1, Math.max(1, draft.y - 1), 1);
  if (wraparound) {
    return wraparound;
  }

  for (let row = 301; row <= 600; row++) {
    for (let col = 1; col <= maxColStart; col++) {
      const candidate = { ...draft, x: col, y: row };
      const collision = widgets.some((widget) => {
        if (isLayoutWidgetKind(widget.kind)) return false;
        return layoutsOverlap(widget.layout, candidate);
      });
      if (!collision) {
        return candidate;
      }
    }
  }

  return { ...draft, x: 1, y: 1 };
}

function normalizeWidget(raw: DashboardWidget): DashboardWidget {
  return {
    ...raw,
    layout: clampLayout(raw.layout),
    parentLayoutId:
      typeof raw.parentLayoutId === 'string' && raw.parentLayoutId.trim().length > 0
        ? raw.parentLayoutId
        : undefined,
    config:
      raw.config && typeof raw.config === 'object' && !Array.isArray(raw.config)
        ? ({ ...raw.config } as WidgetConfigRecord)
        : undefined
  };
}

function normalizeTab(tab: DashboardTab): DashboardTab {
  return {
    ...tab,
    widgets: tab.widgets.map(normalizeWidget)
  };
}

function isObject(value: unknown): value is Record<string, unknown> {
  return !!value && typeof value === 'object' && !Array.isArray(value);
}

export function normalizeLayoutState(raw: unknown): DashboardLayoutState | null {
  if (!isObject(raw)) return null;
  const parsed = raw as Partial<DashboardLayoutState>;
  if (!Array.isArray(parsed.tabs) || parsed.tabs.length === 0) return null;

  const tabs = parsed.tabs
    .filter((tab): tab is DashboardTab => !!tab && typeof tab.id === 'string' && Array.isArray(tab.widgets))
    .map((tab) =>
      normalizeTab({
        id: tab.id,
        name: typeof tab.name === 'string' ? tab.name : 'Dashboard',
        widgets: tab.widgets.filter(
          (widget): widget is DashboardWidget =>
            !!widget &&
            typeof widget.id === 'string' &&
            typeof widget.signalId === 'number' &&
            typeof widget.kind === 'string' &&
            typeof widget.title === 'string' &&
            !!widget.layout &&
            typeof widget.layout.x === 'number' &&
            typeof widget.layout.y === 'number' &&
            typeof widget.layout.w === 'number' &&
            typeof widget.layout.h === 'number' &&
            (widget.parentLayoutId === undefined ||
              (typeof widget.parentLayoutId === 'string' &&
                widget.parentLayoutId.trim().length > 0)) &&
            (widget.config === undefined ||
              (widget.config && typeof widget.config === 'object' && !Array.isArray(widget.config)))
        )
      })
    );

  if (tabs.length === 0) return null;

  const activeTabId =
    typeof parsed.activeTabId === 'string' && tabs.some((tab) => tab.id === parsed.activeTabId)
      ? parsed.activeTabId
      : tabs[0].id;

  return { activeTabId, tabs };
}

export function toLayoutExport(layout: DashboardLayoutState): DashboardLayoutExport {
  return {
    format: LAYOUT_EXPORT_FORMAT,
    version: LAYOUT_EXPORT_VERSION,
    exported_at: new Date().toISOString(),
    layout
  };
}

export function serializeLayoutExport(layout: DashboardLayoutState): string {
  return JSON.stringify(toLayoutExport(layout), null, 2);
}

export function parseLayoutImport(raw: string): DashboardLayoutState {
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch (err) {
    throw new Error(`invalid JSON: ${String(err)}`);
  }

  const layoutCandidate =
    isObject(parsed) && 'layout' in parsed ? (parsed as { layout: unknown }).layout : parsed;
  const normalized = normalizeLayoutState(layoutCandidate);
  if (!normalized) {
    throw new Error('file does not contain a valid dashboard layout');
  }
  return normalized;
}

export function createEmptyTab(name?: string): DashboardTab {
  return {
    id: nextId('tab'),
    name: name ?? 'Dashboard',
    widgets: []
  };
}

export function createDefaultLayout(): DashboardLayoutState {
  const tab = createEmptyTab('Main');
  return {
    activeTabId: tab.id,
    tabs: [tab]
  };
}

export function loadLayout(): DashboardLayoutState {
  try {
    const raw = localStorage.getItem(LAYOUT_STORAGE_KEY);
    if (raw) {
      const normalized = normalizeLayoutState(JSON.parse(raw));
      if (normalized) return normalized;
    }
  } catch {
    // ignore and fall back
  }

  // Migrate from pre-tab widget store if present.
  try {
    const raw = localStorage.getItem(LEGACY_WIDGET_STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw) as DashboardWidget[];
      if (Array.isArray(parsed) && parsed.length > 0) {
        const migratedWidgets: DashboardWidget[] = [];
        for (const item of parsed) {
          if (!item || typeof item.signalId !== 'number' || typeof item.kind !== 'string') continue;
          const kind = item.kind as WidgetKind;
          const base = defaultLayoutFor(kind);
          const layout = findNextWidgetPlacement(migratedWidgets, base);
          migratedWidgets.push({
            id: typeof item.id === 'string' ? item.id : nextId('widget'),
            signalId: item.signalId,
            kind,
            title: typeof item.title === 'string' ? item.title : `Signal ${item.signalId}`,
            layout
          });
        }

        const tab = createEmptyTab('Main');
        tab.widgets = migratedWidgets;
        const state: DashboardLayoutState = {
          activeTabId: tab.id,
          tabs: [tab]
        };
        saveLayout(state);
        localStorage.removeItem(LEGACY_WIDGET_STORAGE_KEY);
        return state;
      }
    }
  } catch {
    // ignore
  }

  return createDefaultLayout();
}

export function saveLayout(layout: DashboardLayoutState) {
  try {
    localStorage.setItem(LAYOUT_STORAGE_KEY, JSON.stringify(layout));
  } catch {
    // Ignore local storage failures.
  }
}

export function makeWidget(
  signal: SignalRow,
  kind: WidgetKind,
  existingWidgets: DashboardWidget[],
  titleOverride?: string,
  columns = DEFAULT_GRID_COLUMNS,
  preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
  parentLayoutId?: string,
  config?: WidgetConfigRecord
): DashboardWidget {
  const desiredBase = defaultLayoutFor(kind);
  const desired = preferredPosition
    ? { ...desiredBase, x: preferredPosition.x, y: preferredPosition.y }
    : desiredBase;
  const layout = findNextWidgetPlacement(existingWidgets, desired, columns);
  return {
    id: nextId('widget'),
    signalId: signal.signal_id,
    kind,
    title: titleOverride?.trim() || defaultSignalWidgetTitle(signal, kind),
    layout,
    parentLayoutId:
      typeof parentLayoutId === 'string' && parentLayoutId.trim().length > 0
        ? parentLayoutId
        : undefined,
    config:
      config && typeof config === 'object' && !Array.isArray(config)
        ? ({ ...config } as WidgetConfigRecord)
        : undefined
  };
}

export function makeLayoutWidget(
  kind: LayoutToolKind,
  existingWidgets: DashboardWidget[],
  titleOverride?: string,
  columns = DEFAULT_GRID_COLUMNS,
  preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
  config?: WidgetConfigRecord,
  parentLayoutId?: string
): DashboardWidget {
  const desiredBase = defaultLayoutFor(kind);
  const desired = preferredPosition
    ? { ...desiredBase, x: preferredPosition.x, y: preferredPosition.y }
    : desiredBase;
  const layout = findNextWidgetPlacement(existingWidgets, desired, columns);

  return {
    id: nextId('widget'),
    signalId: 0,
    kind,
    title: titleOverride?.trim() || defaultLayoutTitle(kind),
    layout,
    parentLayoutId:
      typeof parentLayoutId === 'string' && parentLayoutId.trim().length > 0
        ? parentLayoutId
        : undefined,
    config: (() => {
      if (config && typeof config === 'object' && !Array.isArray(config)) {
        return { ...config } as WidgetConfigRecord;
      }
      if (kind === 'layout_title') {
        return {
          text: titleOverride?.trim() || defaultLayoutTitle(kind),
          color: '#f5f7fb'
        } as WidgetConfigRecord;
      }
      if (kind === 'layout_grid') {
        return {
          columns: layout.w,
          rows: layout.h,
          autoSize: false
        } as WidgetConfigRecord;
      }
      return undefined;
    })()
  };
}

export function formatUptime(ms: number): string {
  const s = Math.floor(ms / 1000);
  const m = Math.floor(s / 60);
  const rem = s % 60;
  return `${m}m ${rem}s`;
}

export function formatMemory(bytes: number | null): string {
  if (bytes === null) return 'n/a';
  return `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
}

export function formatCpu(cpu: number | null): string {
  if (cpu === null) return 'n/a';
  return `${cpu.toFixed(1)}%`;
}

export function parseNumericSignal(signal: SignalRow): number | null {
  if (signal.signal_type !== 'f64' && signal.signal_type !== 'i64') {
    return null;
  }
  const value = Number(signal.value);
  return Number.isFinite(value) ? value : null;
}

export function sparklinePath(values: number[]): string {
  if (values.length < 2) return 'M 0 20 L 100 20';

  let min = Infinity;
  let max = -Infinity;
  for (const value of values) {
    if (value < min) min = value;
    if (value > max) max = value;
  }
  const range = Math.max(1e-9, max - min);

  return values
    .map((value, index) => {
      const x = (index / (values.length - 1)) * 100;
      const y = 34 - ((value - min) / range) * 30 - 2;
      return `${index === 0 ? 'M' : 'L'} ${x.toFixed(2)} ${y.toFixed(2)}`;
    })
    .join(' ');
}

export function applyFilters(
  signals: SignalRow[],
  query: string,
  roleFilter: string,
  typeFilter: string
): SignalRow[] {
  const q = query.trim().toLowerCase();
  return signals.filter((signal) => {
    const role = signalRole(signal);
    if (roleFilter !== 'all' && role !== roleFilter) return false;
    if (typeFilter !== 'all' && signal.signal_type !== typeFilter) return false;
    if (!q) return true;
    return (
      signal.path.toLowerCase().includes(q) ||
      signal.signal_type.toLowerCase().includes(q) ||
      role.toLowerCase().includes(q) ||
      signal.kind.toLowerCase().includes(q) ||
      signal.access.toLowerCase().includes(q) ||
      String(signal.signal_id).includes(q)
    );
  });
}
