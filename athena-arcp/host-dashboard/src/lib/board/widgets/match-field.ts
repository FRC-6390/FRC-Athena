/** Field 2D geometry and pose decoding. Pure. Field coordinates are WPILib metres (origin at the blue corner). */
import type { SignalValue, StructValue } from '../../data/types';
import type { Alliance } from './match-alliance';

/** Field image extent in metres, same geometry as the analysis field view. */
export const FIELD_VIEW = { x: -0.5, y: -0.5, width: 17.541, height: 9.0692 };
export const FIELD_HEIGHT = 8.0692;
/** Centre of the drawn field, used for the red-perspective rotation. */
export const FIELD_CENTRE = { x: FIELD_VIEW.x + FIELD_VIEW.width / 2, y: FIELD_VIEW.y + FIELD_VIEW.height / 2 };
export const ROBOT_SIZE = 0.8;

export type Perspective = 'blue' | 'red' | 'auto';

export type Pose = { x: number; y: number; theta: number };

export type TimedPose = Pose & { timestampMicros: number };

/** Decodes a WPILib Pose2d struct (x, y, rotation: 3 little-endian doubles). Null when malformed. */
export function decodePose2d(value: SignalValue | undefined): Pose | null {
  if (value === undefined || typeof value !== 'object' || Array.isArray(value) || !('schemaId' in value)) return null;
  const struct = value as StructValue;
  if (struct.bytes.length < 24) return null;
  const view = new DataView(struct.bytes.buffer, struct.bytes.byteOffset, 24);
  const pose = { x: view.getFloat64(0, true), y: view.getFloat64(8, true), theta: view.getFloat64(16, true) };
  return Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.theta) ? pose : null;
}

/** Whether the scene is drawn rotated 180 degrees (red alliance side up). */
export function isRotated(perspective: Perspective, alliance: Alliance | null): boolean {
  if (perspective === 'red') return true;
  if (perspective === 'auto') return alliance === 'red';
  return false;
}

/** Screen position in field-image units (y grows downwards). */
export function screenPoint(pose: Pose): { x: number; y: number } {
  return { x: pose.x, y: FIELD_HEIGHT - pose.y };
}

/** SVG rotate() for a heading, accounting for the flipped y axis. */
export function headingDegrees(theta: number): number {
  return (-theta * 180) / Math.PI;
}

/** Poses from the last `windowSeconds` of samples, oldest first. */
export function trailPoses(samples: readonly { timestampMicros: number; value: SignalValue }[], windowSeconds: number): TimedPose[] {
  const decoded: TimedPose[] = [];
  for (const sample of samples) {
    const pose = decodePose2d(sample.value);
    if (pose) decoded.push({ ...pose, timestampMicros: sample.timestampMicros });
  }
  if (decoded.length === 0 || windowSeconds <= 0) return decoded.slice(-1);
  const newest = decoded[decoded.length - 1].timestampMicros;
  const horizon = newest - windowSeconds * 1_000_000;
  return decoded.filter((p) => p.timestampMicros >= horizon);
}

/** SVG polyline points for a trail. */
export function trailPointsAttr(poses: readonly Pose[]): string {
  return poses.map((p) => {
    const s = screenPoint(p);
    return `${s.x.toFixed(3)},${s.y.toFixed(3)}`;
  }).join(' ');
}

export function formatPose(pose: Pose | null): string {
  if (!pose) return 'No pose';
  return `${pose.x.toFixed(2)} m, ${pose.y.toFixed(2)} m, ${((pose.theta * 180) / Math.PI).toFixed(0)}°`;
}
