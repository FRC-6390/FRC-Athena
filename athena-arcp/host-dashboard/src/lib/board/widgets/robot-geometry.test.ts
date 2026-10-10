import { describe, expect, test } from 'bun:test';
import { compassArcPath, compassPoint, fitScale, mapRange, normalizeDegrees, shortestDelta, toDegrees, twoLinkJoints, wrapDegrees } from './robot-geometry';

const near = (a: number, b: number, eps = 1e-6) => expect(Math.abs(a - b)).toBeLessThan(eps);

describe('angles', () => {
  test('wrapDegrees maps into (-180, 180]', () => {
    expect(wrapDegrees(190)).toBe(-170);
    expect(wrapDegrees(-180)).toBe(180);
    expect(wrapDegrees(180)).toBe(180);
    expect(wrapDegrees(725)).toBe(5);
    expect(wrapDegrees(Number.NaN)).toBe(0);
  });
  test('normalizeDegrees maps into [0, 360)', () => {
    expect(normalizeDegrees(-90)).toBe(270);
    expect(normalizeDegrees(360)).toBe(0);
  });
  test('shortestDelta crosses the seam the short way', () => {
    expect(shortestDelta(350, 10)).toBe(20);
    expect(shortestDelta(10, 350)).toBe(-20);
  });
  test('radians convert', () => {
    near(toDegrees(Math.PI / 2, 'rad'), 90);
    expect(toDegrees(45, 'deg')).toBe(45);
  });
  test('mapRange handles degenerate ranges', () => {
    expect(mapRange(5, 0, 10, 0, 100)).toBe(50);
    expect(mapRange(5, 3, 3, 7, 9)).toBe(7);
  });
});

describe('drawing geometry', () => {
  test('compass 0 is up and 90 is right', () => {
    const up = compassPoint(50, 50, 40, 0);
    near(up.x, 50);
    near(up.y, 10);
    const right = compassPoint(50, 50, 40, 90);
    near(right.x, 90);
    near(right.y, 50);
  });
  test('arc path is empty for non-positive spans and flags large arcs', () => {
    expect(compassArcPath(0, 0, 10, 20, 20)).toBe('');
    expect(compassArcPath(0, 0, 10, 0, 90)).toContain(' 0 0 1 ');
    expect(compassArcPath(0, 0, 10, 0, 270)).toContain(' 0 1 1 ');
  });
  test('two-link arm: straight out and elbow folded back', () => {
    const straight = twoLinkJoints({ x: 0, y: 0 }, 0, 0, 1, 1, 10);
    near(straight.tip.x, 20);
    near(straight.tip.y, 0);
    const folded = twoLinkJoints({ x: 0, y: 0 }, 0, 180, 1, 1, 10);
    near(folded.tip.x, 0);
    near(folded.tip.y, 0);
  });
  test('fitScale fits the reach into the box', () => {
    expect(fitScale(1, 220, 120, 10)).toBe(50);
    expect(fitScale(0, 100, 100, 0)).toBe(1);
  });
});
