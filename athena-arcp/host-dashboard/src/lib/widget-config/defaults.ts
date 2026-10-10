import { SignalRow } from '../arcp';
import { WidgetConfigRecord, WidgetKind } from '../dashboard';
import { readCameraOverlayConfig } from './camera';
import { isObject } from './common';
import { readControllerConfig } from './controller';
import { readDioConfig } from './dio';
import { readEncoderConfig } from './encoder';
import { readFieldConfig } from './field';
import { readGraphConfig } from './graph';
import { readImu3dConfig, readImuConfig } from './imu';
import { readBarConfig, readDialConfig, readStatusMatrixConfig, readTimerConfig } from './indicators';
import { readChoiceConfig, readDropdownConfig, readInputConfig, readTextAreaConfig, readToggleConfig } from './inputs';
import { readLayoutAccordionConfig, readLayoutTitleConfig } from './layout';
import { readMech2dConfig } from './mech2d';
import { readMotorConfig } from './motor-encoder';
import { readStateMachineConfig } from './state-machine';
import { readDifferentialDriveConfig, readSwerveDriveConfig, readSwerveModuleConfig } from './swerve';

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
    case 'state_machine':
      return readStateMachineConfig(undefined, signal, signals);
    case 'dio':
      return readDioConfig(undefined, signal, signals);
    case 'motor':
      return readMotorConfig(undefined, signal, signals);
    case 'encoder':
      return readEncoderConfig(undefined, signal, signals);
    case 'imu':
      return readImuConfig(undefined, signal, signals);
    case 'layout_title':
      return readLayoutTitleConfig(undefined, 'Title');
    case 'layout_accordion':
      return readLayoutAccordionConfig(undefined);
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
