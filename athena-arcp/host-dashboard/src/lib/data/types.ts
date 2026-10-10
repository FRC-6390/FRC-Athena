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

/** Signal role (E3). The numeric encoding of the dictionary `u8 role` byte is not fixed by the appendix; see ROLE_BY_CODE. */
export type SignalRole = 'unknown' | 'state' | 'value' | 'control' | 'device' | 'action' | 'marker';

/**
 * Numeric role codes used in DICTIONARY entries. The appendix leaves this mapping open;
 * this is the assumed mapping shared with the Java writer. Unknown codes decode as 'unknown'.
 */
export const ROLE_BY_CODE: readonly SignalRole[] = [
  'unknown',
  'state',
  'value',
  'control',
  'device',
  'action',
  'marker'
];

/**
 * Lock state of a writable value (E3 / A2). Not carried by the wire format today; sources that know
 * it (mock, future schema channel) fill it in. Clients display it but never use it to block writes (E5).
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
  /** Lock group simple name (e.g. `InMatch`), empty when none. */
  lockGroup: string;
  lockState: LockState;
  /** Non-zero for struct-typed signals. */
  structSchemaId: number;
  /** Mechanism path owning this signal (the segments before the first group segment). */
  mechanismPath: string;
  /** Path segments after the mechanism path, starting with the group name (e.g. `Controls/position/kP`). */
  localPath: string;
  /** Last path segment. */
  name: string;
};

/** Raw values as decoded from FRAME entries. */
export type StructValue = {
  schemaId: number;
  bytes: Uint8Array;
};

export type SignalValue =
  | boolean
  | number
  | string
  | boolean[]
  | number[]
  | string[]
  | StructValue;

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
