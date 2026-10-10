import { hardwareWidgetKindForPath, isWritableSignal, signalRole } from '../dashboard';
import type { SignalRow } from '../arcp';
import { accessMatches, typeMatches, type CompatSignal, type WidgetModule } from './contract';
import { PLOT_MODULE } from './plot';
import { SIGNAL_WIDGET_KINDS, type SignalWidgetKind } from './kinds';

/**
 * The widget registry: the single source of truth for which widget kinds exist and which of them a signal
 * can be shown with. Classic UI, palette, inspector and the config editor all go through here.
 */
const SCALAR_OR_ARRAY_WRITABLE = ['bool', 'string', 'numeric', 'bool[]', 'string[]'] as const;
const NOT_ACTION = ['action'] as const;

function legacy(
  id: SignalWidgetKind,
  title: string,
  accepts: WidgetModule['accepts'],
  access: WidgetModule['access'],
  extra: Partial<WidgetModule> = {}
): WidgetModule {
  return { id, title, summary: title, accepts, access, excludeRoles: NOT_ACTION, legacy: true, ...extra };
}

/** Classic modules in palette order (no-signal palette order first, then the signal-only kinds). */
const LEGACY_MODULES: WidgetModule[] = [
  legacy('metric', 'Metric', ['bool', 'numeric'], 'any'),
  legacy('state', 'State', ['bool', 'string', 'string[]', 'bool[]', 'f64[]', 'i64[]'], 'any', {
    excludeRoles: undefined,
    acceptsRoles: ['action']
  }),
  legacy('text', 'Text', ['*'], 'any', { excludeRoles: undefined }),
  legacy('trend', 'Trend', ['numeric'], 'any'),
  legacy('state_machine', 'State machine', ['*'], 'any', {
    pathHint: (path) => hardwareWidgetKindForPath(path) === 'state_machine'
  }),
  legacy('dio', 'Digital I/O', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'dio' }),
  legacy('motor', 'Motor', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'motor' }),
  legacy('encoder', 'Encoder', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'encoder' }),
  legacy('imu', 'IMU', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'imu' }),
  legacy('graph', 'Graph', ['numeric'], 'any'),
  legacy('bar', 'Bar', ['numeric'], 'any'),
  legacy('dial', 'Dial', ['numeric'], 'any'),
  legacy('compass', 'Compass', ['numeric'], 'any'),
  legacy('timer', 'Timer', ['numeric'], 'any'),
  legacy('field', 'Field', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('imu_3d', '3D orientation', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('mech2d', 'Mechanism 2D', ['string', 'string[]'], 'any'),
  legacy('swerve_module', 'Swerve module', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('swerve_drive', 'Swerve drive', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('differential_drive', 'Differential drive', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('dropdown', 'Dropdown', [...SCALAR_OR_ARRAY_WRITABLE], 'write'),
  legacy('status_matrix', 'Status matrix', ['bool'], 'any'),
  legacy('input', 'Input', ['string', 'string[]', 'numeric'], 'write'),
  legacy('textarea', 'Text area', ['string', 'string[]'], 'write'),
  legacy('radio', 'Radio', ['bool', 'string', 'string[]'], 'write'),
  legacy('button_group', 'Button group', ['bool', 'string', 'string[]'], 'write'),
  legacy('toggle', 'Toggle', ['bool'], 'write'),
  legacy('tunable', 'Tunable', [...SCALAR_OR_ARRAY_WRITABLE], 'write', { offeredWithoutSignal: false }),
  legacy('controller', 'Controller', ['numeric'], 'any', { offeredWithoutSignal: false }),
  legacy('camera_overlay', 'Camera overlay', ['string', 'string[]', 'numeric'], 'any', { offeredWithoutSignal: false }),
  legacy('action', 'Action', ['*'], 'any', { roles: ['action'], excludeRoles: undefined, offeredWithoutSignal: false })
];

/** Every widget module, in palette order. */
export const WIDGET_MODULES: readonly WidgetModule[] = [...LEGACY_MODULES, PLOT_MODULE];

const BY_ID = new Map<string, WidgetModule>(WIDGET_MODULES.map((m) => [m.id, m]));

/** Each classic kind must have exactly one module; this fails at import time if the lists drift apart. */
for (const kind of SIGNAL_WIDGET_KINDS) {
  if (BY_ID.get(kind)?.legacy !== true) {
    throw new Error(`widget registry is missing classic module '${kind}'`);
  }
}

export function widgetModule(id: string): WidgetModule | undefined {
  return BY_ID.get(id);
}

/** Adapts a classic signal row to the shape the registry matches against. */
export function compatSignalFromRow(row: SignalRow): CompatSignal {
  return {
    path: row.path,
    type: row.signal_type,
    role: signalRole(row),
    writable: isWritableSignal(row)
  };
}

/** Modules that can display this signal, in palette order (all registered modules, plot included). */
export function compatibleModules(signal: CompatSignal): WidgetModule[] {
  return WIDGET_MODULES.filter((module) => {
    if (module.roles && !module.roles.includes(signal.role)) return false;
    if (module.excludeRoles?.includes(signal.role)) return false;
    if (!accessMatches(module.access, signal.writable)) return false;
    const typeOk =
      module.accepts.some((pattern) => typeMatches(pattern, signal.type)) ||
      (module.acceptsRoles?.includes(signal.role) ?? false);
    if (!typeOk) return false;
    if (module.pathHint && !module.pathHint(signal.path)) return false;
    return true;
  });
}

/**
 * Classic palette: the classic kinds to offer for the selected signal (or for no selection). This is the
 * only compatibility rule the classic UI uses.
 */
export function classicKindsFor(row: SignalRow | null): SignalWidgetKind[] {
  if (!row) {
    return WIDGET_MODULES.filter((m) => m.legacy && m.offeredWithoutSignal !== false).map(
      (m) => m.id as SignalWidgetKind
    );
  }
  return compatibleModules(compatSignalFromRow(row))
    .filter((m) => m.legacy)
    .map((m) => m.id as SignalWidgetKind);
}
