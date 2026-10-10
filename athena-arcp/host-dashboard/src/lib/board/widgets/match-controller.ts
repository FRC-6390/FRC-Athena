/**
 * Gamepad viewer mapping. Joystick signals live under `input/joystick/<port>/`; this maps their local names
 * (`a`, `leftBumper`, `leftX`, `buttons/a`, ...) onto the controller layout. Pure.
 */

export const BUTTON_IDS = [
  'a',
  'b',
  'x',
  'y',
  'leftBumper',
  'rightBumper',
  'back',
  'start',
  'leftStick',
  'rightStick',
  'dpadUp',
  'dpadDown',
  'dpadLeft',
  'dpadRight'
] as const;

export const AXIS_IDS = ['leftX', 'leftY', 'rightX', 'rightY', 'leftTrigger', 'rightTrigger'] as const;

export type ButtonId = (typeof BUTTON_IDS)[number];
export type AxisId = (typeof AXIS_IDS)[number];

export type JoystickMap = {
  buttons: Partial<Record<ButtonId, string>>;
  axes: Partial<Record<AxisId, string>>;
  /** Signals on this port that do not map to a control. */
  unmapped: string[];
};

export function joystickPrefix(port: number): string {
  return `input/joystick/${port}/`;
}

export function isJoystickPath(path: string): boolean {
  return path.startsWith('input/joystick/');
}

/** Lower-case key with separators removed, so `left_bumper`, `LeftBumper` and `leftbumper` match. */
function normalise(name: string): string {
  return name.toLowerCase().replace(/[^a-z0-9]/g, '');
}

export function mapJoystick(paths: readonly string[], port: number): JoystickMap {
  const prefix = joystickPrefix(port);
  const buttonByKey = new Map<string, ButtonId>(BUTTON_IDS.map((id) => [normalise(id), id]));
  const axisByKey = new Map<string, AxisId>(AXIS_IDS.map((id) => [normalise(id), id]));
  const map: JoystickMap = { buttons: {}, axes: {}, unmapped: [] };

  for (const path of paths) {
    if (!path.startsWith(prefix)) continue;
    const local = path.slice(prefix.length);
    const leaf = normalise(local.split('/').pop() ?? local);
    const button = buttonByKey.get(leaf);
    const axis = axisByKey.get(leaf);
    if (button && !map.buttons[button]) map.buttons[button] = path;
    else if (axis && !map.axes[axis]) map.axes[axis] = path;
    else map.unmapped.push(path);
  }
  map.unmapped.sort();
  return map;
}

/** Stick and trigger value clamped to its range: sticks -1..1, triggers 0..1. */
export function axisValue(value: unknown, axis: AxisId): number {
  const n = typeof value === 'bigint' ? Number(value) : value;
  if (typeof n !== 'number' || !Number.isFinite(n)) return 0;
  const limit = axis.endsWith('Trigger') ? [0, 1] : [-1, 1];
  return Math.min(limit[1], Math.max(limit[0], n));
}
