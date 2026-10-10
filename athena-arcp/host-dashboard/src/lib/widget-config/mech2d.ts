import { WidgetConfigRecord } from '../dashboard';
import { clamp, isObject, toBoolean, toNumber, toString } from './common';

export type Mech2dWidgetConfig = {
  showGrid: boolean;
  lineWidth: number;
};

export type Mech2dSegment = {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  color: string;
  width: number;
};

export type Mech2dJoint = {
  x: number;
  y: number;
  r: number;
  color: string;
};

export type Mech2dScene = {
  viewportWidth: number;
  viewportHeight: number;
  segments: Mech2dSegment[];
  joints: Mech2dJoint[];
};

export function readMech2dConfig(raw: WidgetConfigRecord | undefined): Mech2dWidgetConfig {
  const base = isObject(raw) ? raw : {};
  return {
    showGrid: toBoolean(base.showGrid, true),
    lineWidth: clamp(toNumber(base.lineWidth) ?? 0.024, 0.002, 0.25)
  };
}

export function parseMech2dSegment(value: unknown): Mech2dSegment | null {
  if (!isObject(value)) return null;
  const x1 = toNumber(value.x1);
  const y1 = toNumber(value.y1);
  const x2 = toNumber(value.x2);
  const y2 = toNumber(value.y2);
  if (x1 === null || y1 === null || x2 === null || y2 === null) return null;
  return {
    x1,
    y1,
    x2,
    y2,
    color: toString(value.color, '#f87171'),
    width: clamp(toNumber(value.width) ?? 0.024, 0.002, 0.25)
  };
}

export function parseMech2dJoint(value: unknown): Mech2dJoint | null {
  if (!isObject(value)) return null;
  const x = toNumber(value.x);
  const y = toNumber(value.y);
  if (x === null || y === null) return null;
  return {
    x,
    y,
    r: clamp(toNumber(value.r) ?? 0.018, 0.002, 0.3),
    color: toString(value.color, '#dbeafe')
  };
}

export function parseMech2dScene(raw: string): Mech2dScene {
  const fallback: Mech2dScene = {
    viewportWidth: 1,
    viewportHeight: 1,
    segments: [],
    joints: []
  };
  const trimmed = raw.trim();
  if (!trimmed) return fallback;

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    if (!isObject(parsed)) return fallback;

    const viewportArray = Array.isArray(parsed.viewport) ? parsed.viewport : null;
    const viewportWidth = Math.max(
      0.1,
      toNumber(viewportArray?.[0] ?? parsed.viewportWidth ?? parsed.width) ?? fallback.viewportWidth
    );
    const viewportHeight = Math.max(
      0.1,
      toNumber(viewportArray?.[1] ?? parsed.viewportHeight ?? parsed.height) ?? fallback.viewportHeight
    );

    const segments = Array.isArray(parsed.segments)
      ? parsed.segments
          .map((entry) => parseMech2dSegment(entry))
          .filter((entry): entry is Mech2dSegment => entry !== null)
          .slice(0, 512)
      : [];

    const joints = Array.isArray(parsed.joints)
      ? parsed.joints
          .map((entry) => parseMech2dJoint(entry))
          .filter((entry): entry is Mech2dJoint => entry !== null)
          .slice(0, 512)
      : [];

    return {
      viewportWidth,
      viewportHeight,
      segments,
      joints
    };
  } catch {
    return fallback;
  }
}
