import type { Component } from 'svelte';
import type { SignalWidgetKind } from '../../lib/widgets/kinds';
import type { DashboardWidget } from '../../lib/dashboard';
import type { LayoutSectionProps, SignalSectionProps } from './types';
import GraphConfigSection from './GraphConfigSection.svelte';
import ControllerConfigSection from './ControllerConfigSection.svelte';
import StateMachineConfigSection from './StateMachineConfigSection.svelte';
import MotorConfigSection from './MotorConfigSection.svelte';
import DioConfigSection from './DioConfigSection.svelte';
import EncoderConfigSection from './EncoderConfigSection.svelte';
import ImuConfigSection from './ImuConfigSection.svelte';
import IndicatorConfigSection from './IndicatorConfigSection.svelte';
import StatusMatrixConfigSection from './StatusMatrixConfigSection.svelte';
import CameraOverlayConfigSection from './CameraOverlayConfigSection.svelte';
import ControlConfigSection from './ControlConfigSection.svelte';
import FieldConfigSection from './FieldConfigSection.svelte';
import SwerveConfigSection from './SwerveConfigSection.svelte';
import LayoutConfigSection from './LayoutConfigSection.svelte';

/**
 * Config sections per widget kind. Each kind's section is looked up here, so the registry of kinds and the
 * editor cannot drift apart (the Partial type is limited to classic kinds and layout blocks).
 */
export const SIGNAL_CONFIG_SECTIONS: Partial<Record<SignalWidgetKind, Component<SignalSectionProps>>> = {
  graph: GraphConfigSection,
  controller: ControllerConfigSection,
  state_machine: StateMachineConfigSection,
  motor: MotorConfigSection,
  dio: DioConfigSection,
  encoder: EncoderConfigSection,
  imu: ImuConfigSection,
  bar: IndicatorConfigSection,
  dial: IndicatorConfigSection,
  imu_3d: IndicatorConfigSection,
  timer: IndicatorConfigSection,
  status_matrix: StatusMatrixConfigSection,
  camera_overlay: CameraOverlayConfigSection,
  dropdown: ControlConfigSection,
  toggle: ControlConfigSection,
  button_group: ControlConfigSection,
  radio: ControlConfigSection,
  input: ControlConfigSection,
  textarea: ControlConfigSection,
  field: FieldConfigSection,
  mech2d: FieldConfigSection,
  swerve_module: SwerveConfigSection,
  swerve_drive: SwerveConfigSection,
  differential_drive: SwerveConfigSection
};

export const LAYOUT_CONFIG_SECTIONS: Partial<Record<DashboardWidget['kind'], Component<LayoutSectionProps>>> = {
  layout_title: LayoutConfigSection,
  layout_accordion: LayoutConfigSection,
  layout_grid: LayoutConfigSection
};
