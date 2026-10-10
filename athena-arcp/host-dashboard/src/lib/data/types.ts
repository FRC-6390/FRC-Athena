/**
 * Client model for the ARCP v2 data layer (design appendix E1-W, section E3).
 *
 * Wire values are decoded into these shapes; UI code consumes only these types and
 * never touches bytes or the v1 snapshot types.
 */

/** Numeric type tags from appendix E1-W "typeTag values". */
export const TYPE_TAG = {
  bool: 1,
  i64: 2,
  f64: 3,
  string: 4,
  'bool[]': 5,
  'i64[]': 6,
  'f64[]': 7,
  'string[]': 8,
  struct: 9
} as const;

export type TypeTag = (typeof TYPE_TAG)[keyof typeof TYPE_TAG];

/** Scalar and array signal types. Structs are spelled `struct:<TypeName>` (e.g. `struct:Pose2d`). */
export type SignalType =
  | 'bool'
  | 'i64'
  | 'f64'
  | 'string'
  | 'bool[]'
  | 'i64[]'
  | 'f64[]'
  | 'string[]'
  | `struct:${string}`;

/**
 * Signal role (E1-W "Dictionary role values", normative). Codes 0-8 map to these names; codes above 8
 * decode as `unspecified` so newer writers stay readable.
 */
export type SignalRole =
  | 'unspecified'
  | 'state'
  | 'value'
  | 'control'
  | 'device'
  | 'action'
  | 'hook'
  | 'lock'
  | 'system';

/** Dictionary role code to role name. The index is the wire value. */
export const ROLE_BY_CODE: readonly SignalRole[] = [
  'unspecified',
  'state',
  'value',
  'control',
  'device',
  'action',
  'hook',
  'lock',
  'system'
];

/** Path prefix of boolean lock-state signals (E1-W "Lock state on the wire"). */
export const LOCK_PATH_PREFIX = 'athena/locks/';

/**
 * Robot-level system signals (role `system`). Mechanism signals are described in `mechanisms/model.ts`.
 * `athena/robot/*` and `athena/power/*` are published by the robot's ARCP bridge (athena-wpilib, ArcpSystemSignals).
 */
export const ROBOT_PATHS = {
  /** DriverStation mode: `Disabled`, `Autonomous`, `Teleop` or `Test`. */
  mode: 'athena/robot/mode',
  /** Seconds left in the current match period, -1 when no period is running. */
  matchTime: 'athena/robot/matchTime',
  /** `Red`, `Blue`, or an empty string when unknown. */
  alliance: 'athena/robot/alliance',
  /** Measured battery (bus) voltage, volts. */
  batteryVolts: 'athena/power/battery/voltage',
  /** Total requested current from the power pass, amps. */
  busDemandAmps: 'athena/power/bus/demandAmps',
  /** True while the power budget is shedding current. */
  busShedding: 'athena/power/bus/shedding'
} as const;

/**
 * Lock state of a writable signal, derived by the store from the role-7 signal at
 * `athena/locks/<lockGroup>`. `unknown` when that signal has not been seen. Clients display it but never
 * use it to block writes (E5).
 */
export type LockState = 'unknown' | 'open' | 'locked';

/** One DICTIONARY entry, enriched with the derived mechanism path and display name. */
export type SignalDescriptor = {
  signalId: number;
  /** Full mechanism-tree path, e.g. `robot/turret/Controls/position/kP`. */
  path: string;
  type: SignalType;
  typeTag: TypeTag;
  unit: string;
  role: SignalRole;
  writable: boolean;
  persisted: boolean;
  /** Fully qualified lock group class name as written on the wire (empty when none). */
  lockGroup: string;
  /** Non-zero for struct-typed signals. */
  structSchemaId: number;
  /** Mechanism path owning this signal (the segments before the first group segment). */
  mechanismPath: string;
  /** Path segments after the mechanism path, starting with the group name (e.g. `Controls/position/kP`). */
  localPath: string;
  /** Last path segment. */
  name: string;
};

/** Struct-typed value: schema id plus the WPILib struct bytes. */
export type StructValue = {
  schemaId: number;
  bytes: Uint8Array;
};

/**
 * i64 values are numbers while they are exact (|v| <= 2^53 - 1) and bigints otherwise, so precision is
 * never lost silently. Use `toNumber` before arithmetic that tolerates rounding.
 */
export type I64Value = number | bigint;

export type SignalValue =
  | boolean
  | number
  | bigint
  | string
  | boolean[]
  | I64Value[]
  | number[]
  | string[]
  | StructValue;

/** Converts an i64 (or any signal number) to a JavaScript number. */
export function toNumber(value: number | bigint): number {
  return typeof value === 'bigint' ? Number(value) : value;
}

/** One timestamped sample. `timestampMicros` is the robot FPGA clock. */
export type Sample = {
  timestampMicros: number;
  value: SignalValue;
};

export type FrameEntry = {
  signalId: number;
  typeTag: TypeTag;
  value: SignalValue;
};

export type Frame = {
  version: number;
  keyframe: boolean;
  fragmented: boolean;
  cycleTimestampMicros: number;
  sequence: number;
  fragmentIndex: number;
  fragmentCount: number;
  entries: FrameEntry[];
};

/** MARKER record payload, E1-W marker kinds. */
export const MARKER_KIND = {
  BOOT: 1,
  MATCH_IDENTIFIED: 2,
  AUTO_START: 3,
  TELEOP_START: 4,
  MATCH_END: 5,
  DISABLE: 6,
  FAULT: 7,
  STAGE_CHANGE: 8,
  CHECK: 9,
  OVERRIDE: 10,
  PERSISTED_WRITE: 11,
  RECORD: 12,
  TEST_START: 13,
  USER_BASE: 100
} as const;

export type Marker = {
  timestampMicros: number;
  kind: number;
  /** Human-readable label derived from kind (`markerKindLabel`). */
  label: string;
  message: string;
  /** Signal the marker refers to, or null when none (wire value 0). */
  signalId: number | null;
};

export type StructSchema = {
  schemaId: number;
  typeName: string;
  wpilibSchema: string;
};

export type DictionaryEntry = SignalDescriptor;

/** Decoded records, in stream order. Compressed blocks are expanded into their contents. */
export type DecodedRecord =
  | { kind: 'dictionary'; entries: SignalDescriptor[] }
  | { kind: 'frame'; frame: Frame }
  | { kind: 'marker'; marker: Marker }
  | { kind: 'metadata'; pairs: Record<string, string> }
  | { kind: 'struct_schema'; schema: StructSchema };
