/**
 * Swerve drive vector math. Frame: x forward, y left, angles counter-clockwise from +x (WPILib convention),
 * metres and metres per second. Pure functions.
 */

export const MODULE_NAMES = ['FL', 'FR', 'BL', 'BR'] as const;
export type ModuleName = (typeof MODULE_NAMES)[number];

export type Vec2 = { x: number; y: number };

export type ModuleState = { speed: number; angleRad: number };

export type ModuleSample = Vec2 & { name: ModuleName; vx: number; vy: number };

/** Module centre relative to the robot centre, from wheelbase (front to back) and track width (left to right). */
export function modulePosition(name: ModuleName, wheelbase: number, trackwidth: number): Vec2 {
  const x = (name === 'FL' || name === 'FR' ? 1 : -1) * (wheelbase / 2);
  const y = (name === 'FL' || name === 'BL' ? 1 : -1) * (trackwidth / 2);
  return { x, y };
}

/** Velocity of a module in the robot frame from its speed and steering angle. */
export function moduleVelocity(state: ModuleState): Vec2 {
  return { x: state.speed * Math.cos(state.angleRad), y: state.speed * Math.sin(state.angleRad) };
}

/**
 * Parses a flat `f64[]` of `[speed0, angle0, speed1, angle1, ...]` (module order FL, FR, BL, BR, angles in
 * radians) into module states. Missing or non-finite entries read as zero speed at zero angle.
 */
export function parseModuleStates(values: readonly number[], angleScale = 1): ModuleState[] {
  return MODULE_NAMES.map((_, i) => {
    const speed = values[2 * i];
    const angle = values[2 * i + 1];
    return {
      speed: Number.isFinite(speed) ? speed : 0,
      angleRad: Number.isFinite(angle) ? angle * angleScale : 0
    };
  });
}

/**
 * Chassis velocity from module velocities by least squares. Each module obeys the rigid-body relation
 * `v_i = v + omega x r_i`, so the unknowns are (vx, vy, omega). With fewer than two modules this returns the
 * mean module velocity with zero rotation, which is the best available estimate.
 */
export function chassisVelocity(modules: readonly ModuleSample[]): { vx: number; vy: number; omega: number } {
  const n = modules.length;
  if (n === 0) return { vx: 0, vy: 0, omega: 0 };
  // Normal equations A^T A u = A^T b, where each module contributes rows [1, 0, -y] (for vx) and [0, 1, x] (for vy).
  let a00 = 0, a01 = 0, a02 = 0, a11 = 0, a12 = 0, a22 = 0;
  let b0 = 0, b1 = 0, b2 = 0;
  for (const m of modules) {
    const rx = -m.y;
    const ry = m.x;
    // Row for vx: [1, 0, rx]; row for vy: [0, 1, ry]
    a00 += 1; a02 += rx; a22 += rx * rx; b0 += m.vx; b2 += rx * m.vx;
    a11 += 1; a12 += ry; a22 += ry * ry; b1 += m.vy; b2 += ry * m.vy;
  }
  // a22 was incremented for both rows above, which is exactly A^T A's (2,2) entry.
  const solved = solve3(
    [
      [a00, 0, a02],
      [0, a11, a12],
      [a02, a12, a22]
    ],
    [b0, b1, b2]
  );
  if (!solved) {
    const vx = modules.reduce((s, m) => s + m.vx, 0) / n;
    const vy = modules.reduce((s, m) => s + m.vy, 0) / n;
    return { vx, vy, omega: 0 };
  }
  return { vx: solved[0], vy: solved[1], omega: solved[2] };
}

/** Rotates a robot-frame velocity into field coordinates for a heading in radians. */
export function fieldVelocity(vx: number, vy: number, headingRad: number): Vec2 {
  const c = Math.cos(headingRad);
  const s = Math.sin(headingRad);
  return { x: vx * c - vy * s, y: vx * s + vy * c };
}

/** Solves a 3x3 linear system by Gaussian elimination with partial pivoting. Null when singular. */
export function solve3(m: readonly (readonly number[])[], b: readonly number[]): number[] | null {
  const a = m.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < 3; col += 1) {
    let pivot = col;
    for (let r = col + 1; r < 3; r += 1) {
      if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
    }
    if (Math.abs(a[pivot][col]) < 1e-12) return null;
    [a[col], a[pivot]] = [a[pivot], a[col]];
    for (let r = 0; r < 3; r += 1) {
      if (r === col) continue;
      const f = a[r][col] / a[col][col];
      for (let c = col; c < 4; c += 1) a[r][c] -= f * a[col][c];
    }
  }
  return [a[0][3] / a[0][0], a[1][3] / a[1][1], a[2][3] / a[2][2]];
}
