/**
 * Simulated robot behind the mock data source: a turret (with a hood child mechanism) and a shooter
 * flywheel. Produces plausible State/Values/Actions/Devices/Controls/Hooks signals and applies writes
 * with the same lock rules the robot enforces (A2, E5).
 */
import {
  LOCK_PATH_PREFIX,
  TYPE_TAG,
  type SignalRole,
  type SignalType,
  type SignalValue,
  type TypeTag
} from './types';

export type MockSpec = {
  path: string;
  type: SignalType;
  typeTag: TypeTag;
  unit: string;
  role: SignalRole;
  writable: boolean;
  persisted: boolean;
  lockGroup: string;
};

/** Lock groups are identified by fully qualified class names on the wire (E1-W). */
const IN_MATCH = 'ca.frc6390.athena.mock.InMatch';
const NEVER = 'ca.frc6390.athena.mock.Never';

/** Simple name of a lock group, as shown to people (`InMatch`). */
export function lockGroupSimpleName(group: string): string {
  const dot = group.lastIndexOf('.');
  return dot < 0 ? group : group.slice(dot + 1);
}

function num(path: string, unit: string, role: MockSpec['role'], extra: Partial<MockSpec> = {}): MockSpec {
  return { path, type: 'f64', typeTag: TYPE_TAG.f64, unit, role, writable: false, persisted: false, lockGroup: '', ...extra };
}

function text(path: string, role: MockSpec['role']): MockSpec {
  return { path, type: 'string', typeTag: TYPE_TAG.string, unit: '', role, writable: false, persisted: false, lockGroup: '' };
}

function flag(path: string, role: MockSpec['role'], extra: Partial<MockSpec> = {}): MockSpec {
  return { path, type: 'bool', typeTag: TYPE_TAG.bool, unit: '', role, writable: false, persisted: false, lockGroup: '', ...extra };
}

const gain = (path: string) => num(path, '', 'control', { writable: true, persisted: true, lockGroup: IN_MATCH });
const limit = (path: string, unit: string) =>
  num(path, unit, 'control', { writable: true, persisted: true, lockGroup: NEVER });

/** Every signal the mock publishes, in dictionary order. */
export const MOCK_SPECS: MockSpec[] = [
  text('robot/turret/State/state', 'state'),
  text('robot/turret/State/reason', 'state'),
  text('robot/turret/State/stage', 'state'),
  num('robot/turret/Values/target', 'deg', 'value', { writable: true, persisted: true, lockGroup: IN_MATCH }),
  num('robot/turret/Values/position', 'deg', 'value'),
  flag('robot/turret/Values/atSetpoint', 'value'),
  flag('robot/turret/Actions/home', 'action'),
  flag('robot/turret/Actions/aimAtHub', 'action'),
  num('robot/turret/Devices/motor/current', 'A', 'device'),
  num('robot/turret/Devices/motor/temperature', 'C', 'device'),
  num('robot/turret/Devices/motor/derate', '', 'device'),
  num('robot/turret/Devices/encoder/position', 'deg', 'device'),
  num('robot/turret/Controls/position/target', 'deg', 'control'),
  num('robot/turret/Controls/position/reference', 'deg', 'control'),
  num('robot/turret/Controls/position/measured', 'deg', 'control'),
  gain('robot/turret/Controls/position/kP'),
  gain('robot/turret/Controls/position/kI'),
  gain('robot/turret/Controls/position/kD'),
  gain('robot/turret/Controls/position/kS'),
  gain('robot/turret/Controls/position/kV'),
  limit('robot/turret/Controls/position/Constraints/minPosition', 'deg'),
  limit('robot/turret/Controls/position/Constraints/maxPosition', 'deg'),
  limit('robot/turret/Controls/position/Constraints/maxVelocity', 'deg/s'),
  limit('robot/turret/Controls/position/Constraints/maxAcceleration', 'deg/s2'),
  text('robot/turret/hood/State/state', 'state'),
  num('robot/turret/hood/Values/target', 'deg', 'value', { writable: true, persisted: true, lockGroup: IN_MATCH }),
  num('robot/turret/hood/Values/position', 'deg', 'value'),
  flag('robot/turret/hood/Actions/stow', 'action'),
  num('robot/turret/hood/Devices/encoder/position', 'deg', 'device'),
  text('robot/shooter/State/state', 'state'),
  text('robot/shooter/State/reason', 'state'),
  num('robot/shooter/Values/targetVelocity', 'rpm', 'value', { writable: true, persisted: true, lockGroup: IN_MATCH }),
  flag('robot/shooter/Actions/spinUp', 'action'),
  flag('robot/shooter/Actions/stop', 'action'),
  num('robot/shooter/Devices/motor/current', 'A', 'device'),
  num('robot/shooter/Devices/motor/temperature', 'C', 'device'),
  num('robot/shooter/Devices/motor/derate', '', 'device'),
  num('robot/shooter/Controls/velocity/target', 'rpm', 'control'),
  num('robot/shooter/Controls/velocity/reference', 'rpm', 'control'),
  num('robot/shooter/Controls/velocity/measured', 'rpm', 'control'),
  gain('robot/shooter/Controls/velocity/kP'),
  gain('robot/shooter/Controls/velocity/kV'),
  limit('robot/shooter/Controls/velocity/Constraints/maxAcceleration', 'rpm/s'),
  flag('robot/shooter/Hooks/readyToShoot', 'hook'),
  struct('robot/drive/Values/pose', 'struct:Pose2d', 'value'),
  text('athena/mode/operator', 'system'),
  num('athena/power/battery/voltage', 'V', 'system'),
  num('athena/power/turret/current', 'A', 'system'),
  num('athena/power/shooter/current', 'A', 'system'),
  num('athena/power/drive/current', 'A', 'system')
];

/** Pose2d struct type name and WPILib layout: x, y (metres), rotation (radians), 24 bytes. */
export const POSE2D_SCHEMA_ID = 1;
export const POSE2D_SCHEMA_TEXT = 'double x;double y;double rot';

/** Struct-typed mock signal (E1-W typeTag 9). */
function struct(path: string, type: `struct:${string}`, role: SignalRole): MockSpec {
  return { path, type, typeTag: TYPE_TAG.struct, unit: '', role, writable: false, persisted: false, lockGroup: '' };
}

/** Encodes a Pose2d in WPILib struct layout. */
export function encodePose2d(x: number, y: number, rotation: number): Uint8Array {
  const view = new DataView(new ArrayBuffer(24));
  view.setFloat64(0, x, true);
  view.setFloat64(8, y, true);
  view.setFloat64(16, rotation, true);
  return new Uint8Array(view.buffer);
}

/** Lock-state signal for a group: boolean, role `lock`, read-only (E1-W). */
export function lockSpecFor(group: string): MockSpec {
  return flag(LOCK_PATH_PREFIX + group, 'lock');
}

export const MATCH_PERIOD_SECONDS = 25;
const TURRET_SPEED_LIMIT = 120;
const TEMP_AMBIENT = 32;

/** Result of a command as the simulated robot sees it. */
export type MockResult = { accepted: boolean; reason: string | null };

export class MockRobot {
  time = 0;
  matchActive = false;
  private readonly values = new Map<string, SignalValue>();
  private readonly specs = new Map<string, MockSpec>();
  /** Lock-state signal path to the group it reports. */
  private readonly lockPaths = new Map<string, string>();
  private homingUntil = -1;
  private turretPosition = 0;
  private turretReference = 0;
  private turretVelocity = 0;
  private hoodPosition = 0;
  private shooterVelocity = 0;
  private shooterReference = 0;
  private turretTemp = TEMP_AMBIENT;
  private shooterTemp = TEMP_AMBIENT;
  private nextMatchToggle = MATCH_PERIOD_SECONDS;
  private seed = 7;
  private pose = { x: 1.5, y: 2.0, rotation: 0 };

  readonly specList: readonly MockSpec[];

  constructor(specs: readonly MockSpec[] = MOCK_SPECS) {
    // Every lock group used by a writable signal gets its own role-7 signal, as the robot publishes it.
    const groups = [...new Set(specs.filter((s) => s.writable && s.lockGroup).map((s) => s.lockGroup))];
    const lockSpecs = groups.map(lockSpecFor).filter((spec) => !specs.some((s) => s.path === spec.path));
    this.specList = [...specs, ...lockSpecs];
    for (const spec of this.specList) {
      this.specs.set(spec.path, spec);
      this.values.set(spec.path, defaultValue(spec));
    }
    for (const group of groups) {
      const path = LOCK_PATH_PREFIX + group;
      this.lockPaths.set(path, group);
      this.values.set(path, this.isLocked(group));
    }
    this.values.set('robot/turret/Values/target', 20);
    this.values.set('robot/turret/Controls/position/kP', 4.0);
    this.values.set('robot/turret/Controls/position/kI', 0.0);
    this.values.set('robot/turret/Controls/position/kD', 0.2);
    this.values.set('robot/turret/Controls/position/kS', 0.1);
    this.values.set('robot/turret/Controls/position/kV', 0.02);
    this.values.set('robot/turret/Controls/position/Constraints/minPosition', -170);
    this.values.set('robot/turret/Controls/position/Constraints/maxPosition', 170);
    this.values.set('robot/turret/Controls/position/Constraints/maxVelocity', TURRET_SPEED_LIMIT);
    this.values.set('robot/turret/Controls/position/Constraints/maxAcceleration', 300);
    this.values.set('robot/turret/hood/Values/target', 30);
    this.values.set('robot/shooter/Values/targetVelocity', 0);
    this.values.set('robot/shooter/Controls/velocity/kP', 0.0004);
    this.values.set('robot/shooter/Controls/velocity/kV', 0.00018);
    this.values.set('robot/shooter/Controls/velocity/Constraints/maxAcceleration', 4000);
  }

  spec(path: string): MockSpec | undefined {
    return this.specs.get(path);
  }

  value(path: string): SignalValue {
    return this.values.get(path) ?? '';
  }

  private isLocked(group: string): boolean {
    return group === IN_MATCH && this.matchActive;
  }

  /** Advances the model by `dt` seconds. Returns markers raised during the step (match transitions). */
  step(dt: number): { kind: 'match_start' | 'match_end' }[] {
    this.time += dt;
    const events: { kind: 'match_start' | 'match_end' }[] = [];
    if (this.time >= this.nextMatchToggle) {
      this.matchActive = !this.matchActive;
      this.nextMatchToggle += MATCH_PERIOD_SECONDS;
      events.push({ kind: this.matchActive ? 'match_start' : 'match_end' });
    }
    this.stepTurret(dt);
    this.stepShooter(dt);
    for (const [path, group] of this.lockPaths) this.values.set(path, this.isLocked(group));
    this.stepPower();
    this.stepDrive();
    this.values.set('athena/mode/operator', this.matchActive ? 'Teleop' : 'Disabled');
    return events;
  }

  /** Battery sags with total current; per-consumer currents come from the mechanism models. */
  private stepPower(): void {
    const turret = this.values.get('robot/turret/Devices/motor/current') as number;
    const shooter = this.values.get('robot/shooter/Devices/motor/current') as number;
    const drive = this.matchActive ? 22 + Math.sin(this.time * 2) * 6 : 1.5;
    const total = turret + shooter + drive;
    this.values.set('athena/power/turret/current', turret);
    this.values.set('athena/power/shooter/current', shooter);
    this.values.set('athena/power/drive/current', drive);
    this.values.set('athena/power/battery/voltage', 12.8 - total * 0.012 + this.noise() * 0.2);
  }

  /** Drive pose circles the field centre during a match and holds still otherwise. */
  private stepDrive(): void {
    if (this.matchActive) {
      const angle = this.time * 0.35;
      this.pose = { x: 8.775 + Math.cos(angle) * 3.2, y: 4.03 + Math.sin(angle) * 1.8, rotation: angle + Math.PI / 2 };
    }
    this.values.set('robot/drive/Values/pose', {
      schemaId: POSE2D_SCHEMA_ID,
      bytes: encodePose2d(this.pose.x, this.pose.y, this.pose.rotation)
    });
  }

  private noise(): number {
    this.seed = (this.seed * 16807) % 2147483647;
    return (this.seed / 2147483647 - 0.5) * 0.1;
  }

  private stepTurret(dt: number): void {
    const v = (p: string) => this.values.get(p) as number;
    const homing = this.time < this.homingUntil;
    if (homing) {
      this.values.set('robot/turret/Values/target', 0);
    }
    const minPos = v('robot/turret/Controls/position/Constraints/minPosition');
    const maxPos = v('robot/turret/Controls/position/Constraints/maxPosition');
    const rawTarget = v('robot/turret/Values/target');
    const target = Math.min(maxPos, Math.max(minPos, rawTarget));
    const maxVel = Math.min(TURRET_SPEED_LIMIT, v('robot/turret/Controls/position/Constraints/maxVelocity'));
    const maxAcc = v('robot/turret/Controls/position/Constraints/maxAcceleration');

    // Reference follows the target with a velocity and acceleration limit (trapezoidal-style).
    const refError = target - this.turretReference;
    const desiredVel = Math.max(-maxVel, Math.min(maxVel, refError / 0.25));
    const dv = Math.max(-maxAcc * dt, Math.min(maxAcc * dt, desiredVel - this.turretVelocity));
    this.turretVelocity += dv;
    this.turretReference += this.turretVelocity * dt;
    this.turretPosition += (this.turretReference - this.turretPosition) * Math.min(1, dt * 12);

    const error = target - this.turretPosition;
    this.values.set('robot/turret/Controls/position/target', target);
    this.values.set('robot/turret/Controls/position/reference', this.turretReference);
    this.values.set('robot/turret/Controls/position/measured', this.turretPosition + this.noise());
    this.values.set('robot/turret/Values/position', this.turretPosition);
    this.values.set('robot/turret/Devices/encoder/position', this.turretPosition + this.noise());
    this.values.set('robot/turret/Values/atSetpoint', Math.abs(error) < 0.5 && !homing);

    const current = 2 + Math.abs(this.turretVelocity) * 0.15;
    this.turretTemp += (TEMP_AMBIENT + current * 1.5 - this.turretTemp) * dt * 0.05;
    this.values.set('robot/turret/Devices/motor/current', current);
    this.values.set('robot/turret/Devices/motor/temperature', this.turretTemp);
    this.values.set('robot/turret/Devices/motor/derate', derate(this.turretTemp));

    let reason = 'At setpoint';
    if (homing) reason = 'Homing: target forced to 0';
    else if (rawTarget !== target) reason = 'Guard: at range edge';
    else if (Math.abs(error) >= 0.5) reason = `Moving: error ${error.toFixed(1)} deg`;
    this.values.set('robot/turret/State/state', homing ? 'Homing' : Math.abs(error) >= 0.5 ? 'Tracking' : 'Idle');
    this.values.set('robot/turret/State/reason', reason);
    this.values.set('robot/turret/State/stage', this.turretTemp > 60 ? 'HOT' : 'NORMAL');

    // Hood follows its own target.
    this.hoodPosition += (v('robot/turret/hood/Values/target') - this.hoodPosition) * Math.min(1, dt * 6);
    this.values.set('robot/turret/hood/Values/position', this.hoodPosition);
    this.values.set('robot/turret/hood/Devices/encoder/position', this.hoodPosition + this.noise());
    this.values.set(
      'robot/turret/hood/State/state',
      Math.abs(v('robot/turret/hood/Values/target') - this.hoodPosition) < 0.5 ? 'Idle' : 'Moving'
    );
  }

  private stepShooter(dt: number): void {
    const v = (p: string) => this.values.get(p) as number;
    const target = v('robot/shooter/Values/targetVelocity');
    const maxAcc = v('robot/shooter/Controls/velocity/Constraints/maxAcceleration');
    const step = Math.max(-maxAcc * dt, Math.min(maxAcc * dt, target - this.shooterReference));
    this.shooterReference += step;
    this.shooterVelocity += (this.shooterReference - this.shooterVelocity) * Math.min(1, dt / 0.4);

    const current = 8 + Math.abs(this.shooterVelocity) * 0.02;
    this.shooterTemp += (TEMP_AMBIENT + current * 2 - this.shooterTemp) * dt * 0.03;
    this.values.set('robot/shooter/Controls/velocity/target', target);
    this.values.set('robot/shooter/Controls/velocity/reference', this.shooterReference);
    this.values.set('robot/shooter/Controls/velocity/measured', this.shooterVelocity + this.noise() * 40);
    this.values.set('robot/shooter/Devices/motor/current', current);
    this.values.set('robot/shooter/Devices/motor/temperature', this.shooterTemp);
    this.values.set('robot/shooter/Devices/motor/derate', derate(this.shooterTemp));

    const ready = target > 0 && Math.abs(this.shooterVelocity - target) < 50;
    this.values.set('robot/shooter/Hooks/readyToShoot', ready);
    this.values.set('robot/shooter/State/state', target === 0 ? 'Idle' : ready ? 'Ready' : 'Spinning up');
    this.values.set(
      'robot/shooter/State/reason',
      target === 0 ? 'Stopped' : ready ? 'Within 50 rpm of target' : `Spin-up: ${Math.round(target - this.shooterVelocity)} rpm to go`
    );
  }

  /** Simulated command execution. Mirrors the robot's server-side validation (A2, E5). */
  run(path: string): MockResult {
    const spec = this.specs.get(path);
    if (!spec || spec.role !== 'action') return { accepted: false, reason: 'rejected: not an action' };
    if (path === 'robot/turret/Actions/home') {
      this.homingUntil = this.time + 2;
    } else if (path === 'robot/turret/Actions/aimAtHub') {
      this.values.set('robot/turret/Values/target', 45);
    } else if (path === 'robot/turret/hood/Actions/stow') {
      this.values.set('robot/turret/hood/Values/target', 0);
    } else if (path === 'robot/shooter/Actions/spinUp') {
      this.values.set('robot/shooter/Values/targetVelocity', 3000);
    } else if (path === 'robot/shooter/Actions/stop') {
      this.values.set('robot/shooter/Values/targetVelocity', 0);
    }
    return { accepted: true, reason: null };
  }

  /** Cancels a running action. Turret home and shooter spin-up are the cancellable ones here. */
  cancel(path: string): MockResult {
    const spec = this.specs.get(path);
    if (!spec || spec.role !== 'action') return { accepted: false, reason: 'rejected: not an action' };
    if (path === 'robot/turret/Actions/home') {
      this.homingUntil = -1;
      return { accepted: true, reason: null };
    }
    return { accepted: false, reason: 'rejected: action is not running' };
  }

  write(path: string, value: SignalValue): MockResult {
    const spec = this.specs.get(path);
    if (!spec) return { accepted: false, reason: 'rejected: unknown signal' };
    if (!spec.writable) return { accepted: false, reason: 'rejected: not writable' };
    if (spec.lockGroup && this.isLocked(spec.lockGroup)) {
      return { accepted: false, reason: `rejected: locked (${lockGroupSimpleName(spec.lockGroup)})` };
    }
    const expected = spec.type === 'f64' ? 'number' : spec.type === 'bool' ? 'boolean' : 'string';
    if (typeof value !== expected) {
      return { accepted: false, reason: `rejected: expected ${expected}` };
    }
    this.values.set(path, value);
    return { accepted: true, reason: null };
  }
}

function defaultValue(spec: MockSpec): SignalValue {
  switch (spec.type) {
    case 'bool':
      return false;
    case 'string':
      return '';
    case 'f64':
      return 0;
    default:
      if (spec.type.startsWith('struct:')) return { schemaId: POSE2D_SCHEMA_ID, bytes: encodePose2d(0, 0, 0) };
      return 0;
  }
}

/** Thermal derate: full output below 60 C, falling linearly to 50% at 100 C. */
export function derate(temperature: number): number {
  if (temperature <= 60) return 1;
  return Math.max(0.5, 1 - ((temperature - 60) / 40) * 0.5);
}
