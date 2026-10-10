import { hardwareWidgetKindForPath } from '../dashboard';
import type { SignalDescriptor } from '../data/types';
import { accessMatches, typeMatches, type WidgetModule } from './contract';
import { PLOT_MODULE } from './plot';

/** Signal modules rendered by the classic canvas. Metadata mirrors `widgetKindsFor` in dashboard.ts. */
const SCALAR_OR_ARRAY_WRITABLE = ['bool', 'string', 'numeric', 'bool[]', 'string[]'] as const;
const NOT_ACTION = ['action'] as const;

function legacy(
  id: WidgetModule['id'],
  title: string,
  accepts: WidgetModule['accepts'],
  access: WidgetModule['access'],
  extra: Partial<WidgetModule> = {}
): WidgetModule {
  return { id, title, summary: title, accepts, access, excludeRoles: NOT_ACTION, legacy: true, ...extra };
}

const LEGACY_MODULES: WidgetModule[] = [
  legacy('metric', 'Metric', ['bool', 'numeric'], 'any'),
  legacy('state', 'State', ['bool', 'string', 'string[]', 'bool[]', 'f64[]', 'i64[]'], 'any', { excludeRoles: undefined }),
  legacy('text', 'Text', ['*'], 'any', { excludeRoles: undefined }),
  legacy('trend', 'Trend', ['numeric'], 'any'),
  legacy('action', 'Action', ['*'], 'any', { roles: ['action'], excludeRoles: undefined }),
  legacy('tunable', 'Tunable', [...SCALAR_OR_ARRAY_WRITABLE], 'write'),
  legacy('toggle', 'Toggle', ['bool'], 'write'),
  legacy('button_group', 'Button group', ['bool', 'string'], 'write'),
  legacy('radio', 'Radio', ['bool', 'string'], 'write'),
  legacy('input', 'Input', ['string', 'numeric'], 'write'),
  legacy('textarea', 'Text area', ['string', 'string[]'], 'write'),
  legacy('graph', 'Graph', ['numeric'], 'any'),
  legacy('controller', 'Controller', ['numeric'], 'any'),
  legacy('bar', 'Bar', ['numeric'], 'any'),
  legacy('dial', 'Dial', ['numeric'], 'any'),
  legacy('compass', 'Compass', ['numeric'], 'any'),
  legacy('timer', 'Timer', ['numeric'], 'any'),
  legacy('dropdown', 'Dropdown', [...SCALAR_OR_ARRAY_WRITABLE], 'write'),
  legacy('field', 'Field', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('imu_3d', '3D orientation', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('mech2d', 'Mechanism 2D', ['string', 'string[]'], 'any'),
  legacy('swerve_module', 'Swerve module', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('swerve_drive', 'Swerve drive', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('differential_drive', 'Differential drive', ['numeric', 'f64[]', 'i64[]'], 'any'),
  legacy('status_matrix', 'Status matrix', ['bool'], 'any'),
  legacy('camera_overlay', 'Camera overlay', ['string', 'string[]', 'numeric'], 'any'),
  legacy('state_machine', 'State machine', ['*'], 'any', {
    pathHint: (path) => hardwareWidgetKindForPath(path) === 'state_machine'
  }),
  legacy('dio', 'Digital I/O', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'dio' }),
  legacy('motor', 'Motor', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'motor' }),
  legacy('encoder', 'Encoder', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'encoder' }),
  legacy('imu', 'IMU', ['*'], 'any', { pathHint: (path) => hardwareWidgetKindForPath(path) === 'imu' })
];

/** Every widget module, in palette order. */
export const WIDGET_MODULES: readonly WidgetModule[] = [...LEGACY_MODULES, PLOT_MODULE];

const BY_ID = new Map<string, WidgetModule>(WIDGET_MODULES.map((m) => [m.id, m]));

export function widgetModule(id: string): WidgetModule | undefined {
  return BY_ID.get(id);
}

/** Modules that can display this signal, in palette order. */
export function compatibleModules(signal: SignalDescriptor): WidgetModule[] {
  return WIDGET_MODULES.filter((module) => {
    if (module.roles && !module.roles.includes(signal.role)) return false;
    if (module.excludeRoles?.includes(signal.role)) return false;
    if (!accessMatches(module.access, signal.writable)) return false;
    if (!module.accepts.some((pattern) => typeMatches(pattern, signal.type))) return false;
    if (module.pathHint && !module.pathHint(signal.path)) return false;
    return true;
  });
}
