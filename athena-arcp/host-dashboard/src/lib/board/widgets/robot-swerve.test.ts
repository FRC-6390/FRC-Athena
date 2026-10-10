import { describe, expect, test } from 'bun:test';
import { chassisVelocity, fieldVelocity, modulePosition, moduleVelocity, parseModuleStates, solve3, type ModuleName, type ModuleSample } from './robot-swerve';

const NAMES: ModuleName[] = ['FL', 'FR', 'BL', 'BR'];
function modules(speed: (name: ModuleName) => number, angle: (name: ModuleName) => number, L = 0.6, W = 0.5): ModuleSample[] {
  return NAMES.map((name) => {
    const p = modulePosition(name, L, W);
    const v = moduleVelocity({ speed: speed(name), angleRad: angle(name) });
    return { name, ...p, vx: v.x, vy: v.y };
  });
}
const near = (a: number, b: number) => expect(Math.abs(a - b)).toBeLessThan(1e-9);

describe('swerve', () => {
  test('module layout: FL is front left', () => {
    expect(modulePosition('FL', 0.6, 0.5)).toEqual({ x: 0.3, y: 0.25 });
    expect(modulePosition('BR', 0.6, 0.5)).toEqual({ x: -0.3, y: -0.25 });
  });
  test('straight driving gives pure vx', () => {
    const v = chassisVelocity(modules(() => 2, () => 0));
    near(v.vx, 2);
    near(v.vy, 0);
    near(v.omega, 0);
  });
  test('sideways driving gives pure vy', () => {
    const v = chassisVelocity(modules(() => 1.5, () => Math.PI / 2));
    near(v.vx, 0);
    near(v.vy, 1.5);
    near(v.omega, 0);
  });
  test('rotation in place recovers omega from tangential modules', () => {
    // Each module points tangentially: angle = atan2(x, -y) gives speed omega * r.
    const omega = 2;
    const mods = modules(
      (n) => omega * Math.hypot(modulePosition(n, 0.6, 0.5).x, modulePosition(n, 0.6, 0.5).y),
      (n) => {
        const p = modulePosition(n, 0.6, 0.5);
        return Math.atan2(p.x, -p.y);
      }
    );
    const v = chassisVelocity(mods);
    near(v.omega, omega);
    near(v.vx, 0);
    near(v.vy, 0);
  });
  test('empty and mismatched inputs stay finite', () => {
    expect(chassisVelocity([])).toEqual({ vx: 0, vy: 0, omega: 0 });
    expect(parseModuleStates([1, 0, Number.NaN])[1]).toEqual({ speed: 0, angleRad: 0 });
    expect(parseModuleStates([1, Math.PI])[0]).toEqual({ speed: 1, angleRad: Math.PI });
  });
  test('field velocity rotates by heading', () => {
    const f = fieldVelocity(1, 0, Math.PI / 2);
    near(f.x, 0);
    near(f.y, 1);
  });
  test('solve3 handles a singular system', () => {
    expect(solve3([[0, 0, 0], [0, 0, 0], [0, 0, 0]], [1, 1, 1])).toBeNull();
    const s = solve3([[2, 0, 0], [0, 4, 0], [0, 0, 8]], [2, 4, 8]);
    expect(s).toEqual([1, 1, 1]);
  });
});
