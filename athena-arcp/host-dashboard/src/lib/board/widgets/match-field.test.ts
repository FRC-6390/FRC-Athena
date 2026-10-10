import { describe, expect, test } from 'bun:test';
import { decodePose2d, formatPose, isRotated, headingDegrees, trailPoses, trailPointsAttr } from './match-field';
import { mapJoystick, axisValue, joystickPrefix } from './match-controller';
import { encodePose2d } from '../../data/mock-robot';


describe('field pose', () => {
  test('decodes a Pose2d struct and rejects short or non-struct values', () => {
    const bytes = encodePose2d(3.5, 2, Math.PI / 2);
    expect(decodePose2d({ schemaId: 1, bytes })).toEqual({ x: 3.5, y: 2, theta: Math.PI / 2 });
    expect(decodePose2d({ schemaId: 1, bytes: new Uint8Array(8) })).toBeNull();
    expect(decodePose2d(4)).toBeNull();
    expect(decodePose2d(undefined)).toBeNull();
  });

  test('perspective rotation', () => {
    expect(isRotated('blue', 'red')).toBe(false);
    expect(isRotated('red', 'blue')).toBe(true);
    expect(isRotated('auto', 'red')).toBe(true);
    expect(isRotated('auto', null)).toBe(false);
  });

  test('trail keeps the window and orders oldest first', () => {
    const sample = (t: number, x: number) => ({ timestampMicros: t, value: { schemaId: 1, bytes: encodePose2d(x, 1, 0) } });
    const poses = trailPoses([sample(0, 1), sample(4_000_000, 2), sample(9_000_000, 3)], 5);
    expect(poses.map((p) => p.x)).toEqual([2, 3]);
    expect(trailPointsAttr(poses).split(' ')).toHaveLength(2);
    expect(trailPoses([sample(0, 1)], 0)).toHaveLength(1);
  });

  test('heading and text', () => {
    expect(headingDegrees(Math.PI / 2)).toBeCloseTo(-90);
    expect(formatPose(null)).toBe('No pose');
    expect(formatPose({ x: 1, y: 2, theta: 0 })).toBe('1.00 m, 2.00 m, 0°');
  });
});

describe('joystick mapping', () => {
  test('maps xbox-style names, nested buttons, and reports the rest', () => {
    const p = joystickPrefix(1);
    const map = mapJoystick(
      [`${p}a`, `${p}buttons/left_bumper`, `${p}leftX`, `${p}rightTrigger`, `${p}weird`, 'input/joystick/0/a'],
      1
    );
    expect(map.buttons).toEqual({ a: `${p}a`, leftBumper: `${p}buttons/left_bumper` });
    expect(map.axes).toEqual({ leftX: `${p}leftX`, rightTrigger: `${p}rightTrigger` });
    expect(map.unmapped).toEqual([`${p}weird`]);
  });

  test('axis values are clamped to their range', () => {
    expect(axisValue(2, 'leftX')).toBe(1);
    expect(axisValue(-3, 'leftY')).toBe(-1);
    expect(axisValue(1.4, 'leftTrigger')).toBe(1);
    expect(axisValue(-0.2, 'leftTrigger')).toBe(0);
    expect(axisValue('x', 'leftX')).toBe(0);
  });
});
