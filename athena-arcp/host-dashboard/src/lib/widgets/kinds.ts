/**
 * Saved-layout `kind` strings for signal-bound classic widgets. These values are part of the layout file
 * format; renaming one breaks existing layouts. The registry (`registry.ts`) declares a module for each.
 */
export const SIGNAL_WIDGET_KINDS = [
  'metric',
  'state',
  'text',
  'trend',
  'action',
  'tunable',
  'state_machine',
  'dio',
  'motor',
  'encoder',
  'imu',
  'toggle',
  'button_group',
  'radio',
  'input',
  'textarea',
  'graph',
  'controller',
  'bar',
  'dial',
  'compass',
  'timer',
  'dropdown',
  'field',
  'imu_3d',
  'mech2d',
  'swerve_module',
  'swerve_drive',
  'differential_drive',
  'status_matrix',
  'camera_overlay'
] as const;

export type SignalWidgetKind = (typeof SIGNAL_WIDGET_KINDS)[number];
