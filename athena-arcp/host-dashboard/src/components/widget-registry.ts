import type { Component } from 'svelte';
import type { WidgetComponent, WidgetModuleId } from '../lib/widgets/contract';
import type { SignalWidgetKind } from '../lib/widgets/kinds';
import type { ClassicWidgetProps } from '../lib/widgets/classic-props';
import ActionWidget from './widgets/ActionWidget.svelte';
import BarMeterWidget from './widgets/BarMeterWidget.svelte';
import ButtonGroupWidget from './widgets/ButtonGroupWidget.svelte';
import CameraOverlayWidget from './widgets/CameraOverlayWidget.svelte';
import CompassWidget from './widgets/CompassWidget.svelte';
import ControllerEditorWidget from './widgets/ControllerEditorWidget.svelte';
import DialWidget from './widgets/DialWidget.svelte';
import DifferentialDriveWidget from './widgets/DifferentialDriveWidget.svelte';
import DioWidget from './widgets/DioWidget.svelte';
import DropdownWidget from './widgets/DropdownWidget.svelte';
import EncoderWidget from './widgets/EncoderWidget.svelte';
import FieldViewerWidget from './widgets/FieldViewerWidget.svelte';
import GraphWidget from './widgets/GraphWidget.svelte';
import Imu3dWidget from './widgets/Imu3dWidget.svelte';
import ImuWidget from './widgets/ImuWidget.svelte';
import InputWidget from './widgets/InputWidget.svelte';
import MetricWidget from './widgets/MetricWidget.svelte';
import Mech2dWidget from './widgets/Mech2dWidget.svelte';
import MotorWidget from './widgets/MotorWidget.svelte';
import PlotWidget from './widgets/PlotWidget.svelte';
import RadioWidget from './widgets/RadioWidget.svelte';
import StateMachineWidget from './widgets/StateMachineWidget.svelte';
import StateWidget from './widgets/StateWidget.svelte';
import StatusMatrixWidget from './widgets/StatusMatrixWidget.svelte';
import SwerveDriveWidget from './widgets/SwerveDriveWidget.svelte';
import SwerveModuleWidget from './widgets/SwerveModuleWidget.svelte';
import TextAreaWidget from './widgets/TextAreaWidget.svelte';
import TextWidget from './widgets/TextWidget.svelte';
import TimerWidget from './widgets/TimerWidget.svelte';
import ToggleWidget from './widgets/ToggleWidget.svelte';
import TrendWidget from './widgets/TrendWidget.svelte';
import TunableWidget from './widgets/TunableWidget.svelte';

/**
 * Render components, one per registered widget module. Classic kinds render with the shared
 * `ClassicWidgetProps`; the Record type makes a missing kind a compile error.
 */
export const CLASSIC_COMPONENTS: Record<SignalWidgetKind, Component<ClassicWidgetProps>> = {
  metric: MetricWidget,
  state: StateWidget,
  text: TextWidget,
  trend: TrendWidget,
  action: ActionWidget,
  tunable: TunableWidget,
  state_machine: StateMachineWidget,
  dio: DioWidget,
  motor: MotorWidget,
  encoder: EncoderWidget,
  imu: ImuWidget,
  toggle: ToggleWidget,
  button_group: ButtonGroupWidget,
  radio: RadioWidget,
  input: InputWidget,
  textarea: TextAreaWidget,
  graph: GraphWidget,
  controller: ControllerEditorWidget,
  bar: BarMeterWidget,
  dial: DialWidget,
  compass: CompassWidget,
  timer: TimerWidget,
  dropdown: DropdownWidget,
  field: FieldViewerWidget,
  imu_3d: Imu3dWidget,
  mech2d: Mech2dWidget,
  swerve_module: SwerveModuleWidget,
  swerve_drive: SwerveDriveWidget,
  differential_drive: DifferentialDriveWidget,
  status_matrix: StatusMatrixWidget,
  camera_overlay: CameraOverlayWidget
};

/** Component for a classic kind. Unknown kinds fall back to a plain text readout, as the classic canvas always did. */
export function classicComponentFor(kind: string): Component<ClassicWidgetProps> {
  return (CLASSIC_COMPONENTS as Record<string, Component<ClassicWidgetProps>>)[kind] ?? TextWidget;
}

/** Components for new-contract modules (rendered with `WidgetProps`). */
export const WIDGET_COMPONENTS: Partial<Record<WidgetModuleId, WidgetComponent<never>>> = {
  plot: PlotWidget as unknown as WidgetComponent<never>
};

export function componentFor(id: WidgetModuleId): WidgetComponent<never> | undefined {
  return WIDGET_COMPONENTS[id];
}

