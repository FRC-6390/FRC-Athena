/**
 * Simulated robot behind the mock data source: a turret (with a hood child mechanism) and a shooter
 * flywheel. Publishes signals under the same names the real robot uses (see mechanisms/model.ts) and applies
 * writes with the same lock rules the robot enforces (A2, E5).
 */
import {
  LOCK_PATH_PREFIX,
  ROBOT_PATHS,
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

/** Lock groups are identified by their class names on the wire (E1-W), as the robot publishes them. */
const IN_MATCH = 'ca.frc6390.athena.mechanism.persist.Locks$InMatch';
const CALIBRATION = 'frc.robot.Calibration';

/** Simple name of a lock group, as shown to people (`InMatch`). */
export function lockGroupSimpleName(group: string): string {
  const cut = Math.max(group.lastIndexOf('.'), group.lastIndexOf('$'));
  return cut < 0 ? group : group.slice(cut + 1);
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

/** PID and feedforward gains are writable controls, locked while in match. */
const gain = (path: string) => num(path, '', 'control', { writable: true, persisted: true, lockGroup: IN_MATCH });
/** Constraints are writable controls under the calibration lock. */
const limit = (path: string, unit: string) =>
  num(path, unit, 'control', { writable: true, persisted: true, lockGroup: CALIBRATION });
/** Writable setpoints in Values are role control on the wire. */
const setpoint = (path: string, unit: string) =>
  num(path, unit, 'control', { writable: true, persisted: true, lockGroup: IN_MATCH });

/**
 * Every signal the mock publishes, in dictionary order. Names follow the real robot's dictionary (see
 * `fixtures/real-dictionary.json` and the conventions in `mechanisms/model.ts`).
 */
export const MOCK_SPECS: MockSpec[] = [
  text('robot/turret/State/Reason', 'state'),
  text('robot/turret/State/ActiveAction', 'state'),
  flag('robot/turret/State/ActionRunning', 'state'),
  flag('robot/turret/State/ActionComplete', 'state'),
  text('robot/turret/State/Fault', 'state'),
  setpoint('robot/turret/Values/target', 'deg'),
  flag('robot/turret/Actions/home', 'action'),
  flag('robot/turret/Actions/aimAtHub', 'action'),
  text('robot/turret/Devices/motor/Info/Type', 'device'),
  text('robot/turret/Devices/motor/Info/Kind', 'device'),
  num('robot/turret/Devices/motor/State/SupplyCurrentAmps', 'A', 'device'),
  num('robot/turret/Devices/motor/State/StatorCurrentAmps', 'A', 'device'),
  num('robot/turret/Devices/motor/State/AppliedVoltage', 'V', 'device'),
  num('robot/turret/Devices/motor/State/VelocityRotationsPerSecond', 'rps', 'device'),
  text('robot/turret/Devices/encoder/Info/Type', 'device'),
  num('robot/turret/Devices/encoder/State/Position', 'deg', 'device'),
  text('robot/turret/Controls/position/Info/Type', 'control'),
  text('robot/turret/Controls/position/State/Mode', 'control'),
  num('robot/turret/Controls/position/State/FeedbackPosition', 'deg', 'control'),
  num('robot/turret/Controls/position/State/FeedbackVelocity', 'deg/s', 'control'),
  gain('robot/turret/Controls/position/Config/PID/p'),
  gain('robot/turret/Controls/position/Config/PID/i'),
  gain('robot/turret/Controls/position/Config/PID/d'),
  gain('robot/turret/Controls/position/Config/Feedforward/static'),
  gain('robot/turret/Controls/position/Config/Feedforward/velocity'),
  limit('robot/turret/Controls/position/Config/Constraints/MinimumPosition', 'deg'),
  limit('robot/turret/Controls/position/Config/Constraints/MaximumPosition', 'deg'),
  limit('robot/turret/Controls/position/Config/Constraints/MaximumVelocity', 'deg/s'),
  limit('robot/turret/Controls/position/Config/Constraints/MaximumAcceleration', 'deg/s2'),
  text('robot/turret/hood/State/Reason', 'state'),
  text('robot/turret/hood/State/ActiveAction', 'state'),
  text('robot/turret/hood/State/Fault', 'state'),
  setpoint('robot/turret/hood/Values/target', 'deg'),
  flag('robot/turret/hood/Actions/stow', 'action'),
  text('robot/turret/hood/Devices/encoder/Info/Type', 'device'),
  num('robot/turret/hood/Devices/encoder/State/Position', 'deg', 'device'),
  text('robot/shooter/State/Reason', 'state'),
  text('robot/shooter/State/ActiveAction', 'state'),
  flag('robot/shooter/State/ActionRunning', 'state'),
  flag('robot/shooter/State/ActionComplete', 'state'),
  text('robot/shooter/State/Fault', 'state'),
  setpoint('robot/shooter/Values/targetVelocity', 'rpm'),
  num('robot/shooter/Values/measuredVelocityRps', 'rps', 'value'),
  flag('robot/shooter/Actions/idle', 'action'),
  flag('robot/shooter/Actions/normalPower', 'action'),
  flag('robot/shooter/Actions/stop', 'action'),
  text('robot/shooter/Devices/motor/Info/Type', 'device'),
  text('robot/shooter/Devices/motor/Info/Kind', 'device'),
  num('robot/shooter/Devices/motor/State/SupplyCurrentAmps', 'A', 'device'),
  num('robot/shooter/Devices/motor/State/StatorCurrentAmps', 'A', 'device'),
  num('robot/shooter/Devices/motor/State/VelocityRotationsPerSecond', 'rps', 'device'),
  text('robot/shooter/Controls/velocity/State/Mode', 'control'),
  num('robot/shooter/Controls/velocity/State/FeedbackVelocity', 'rpm', 'control'),
  gain('robot/shooter/Controls/velocity/Config/PID/p'),
  gain('robot/shooter/Controls/velocity/Config/Feedforward/velocity'),
  limit('robot/shooter/Controls/velocity/Config/Constraints/MaximumAcceleration', 'rpm/s'),
  struct('robot/drive/Values/pose', 'struct:Pose2d', 'value'),
  // Robot-level system signals (see ROBOT_PATHS). The robot publishes these through ArcpSystemSignals.
  text(ROBOT_PATHS.mode, 'system'),
  num(ROBOT_PATHS.matchTime, 's', 'system'),
  text(ROBOT_PATHS.alliance, 'system'),
  num(ROBOT_PATHS.batteryVolts, 'V', 'system'),
  num(ROBOT_PATHS.busDemandAmps, 'A', 'system'),
  flag(ROBOT_PATHS.busShedding, 'system')
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
    this.values.set('robot/turret/Controls/position/State/Mode', 'POSITION');
    this.values.set('robot/turret/Controls/position/Info/Type', 'Control');
    this.values.set('robot/turret/Controls/position/Config/PID/p', 4.0);
    this.values.set('robot/turret/Controls/position/Config/PID/i', 0.0);
    this.values.set('robot/turret/Controls/position/Config/PID/d', 0.2);
    this.values.set('robot/turret/Controls/position/Config/Feedforward/static', 0.1);
    this.values.set('robot/turret/Controls/position/Config/Feedforward/velocity', 0.02);
    this.values.set('robot/turret/Controls/position/Config/Constraints/MinimumPosition', -170);
    this.values.set('robot/turret/Controls/position/Config/Constraints/MaximumPosition', 170);
    this.values.set('robot/turret/Controls/position/Config/Constraints/MaximumVelocity', TURRET_SPEED_LIMIT);
    this.values.set('robot/turret/Controls/position/Config/Constraints/MaximumAcceleration', 300);
    this.values.set('robot/turret/hood/Values/target', 30);
    this.values.set('robot/shooter/Values/targetVelocity', 0);
    this.values.set('robot/shooter/Controls/velocity/Config/PID/p', 0.0004);
    this.values.set('robot/shooter/Controls/velocity/Config/Feedforward/velocity', 0.00018);
    this.values.set('robot/shooter/Controls/velocity/Config/Constraints/MaximumAcceleration', 4000);
    this.values.set('robot/shooter/Controls/velocity/State/Mode', 'VELOCITY');
    this.values.set(ROBOT_PATHS.mode, 'Disabled');
    this.values.set(ROBOT_PATHS.matchTime, -1);
    this.values.set(ROBOT_PATHS.alliance, 'Blue');
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
    this.values.set(ROBOT_PATHS.mode, this.matchActive ? 'Teleop' : 'Disabled');
    this.values.set(ROBOT_PATHS.matchTime, this.matchActive ? Math.max(0, this.nextMatchToggle - this.time) : -1);
    return events;
  }

  /** Battery sags with total current. The bus demand is the sum of the motors' supply currents plus the drive. */
  private stepPower(): void {
    const turret = this.values.get('robot/turret/Devices/motor/State/SupplyCurrentAmps') as number;
    const shooter = this.values.get('robot/shooter/Devices/motor/State/SupplyCurrentAmps') as number;
    const drive = this.matchActive ? 22 + Math.sin(this.time * 2) * 6 : 1.5;
    const total = turret + shooter + drive;
    this.values.set(ROBOT_PATHS.busDemandAmps, total);
    this.values.set(ROBOT_PATHS.busShedding, total > 45);
    this.values.set(ROBOT_PATHS.batteryVolts, 12.8 - total * 0.012 + this.noise() * 0.2);
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
    const minPos = v('robot/turret/Controls/position/Config/Constraints/MinimumPosition');
    const maxPos = v('robot/turret/Controls/position/Config/Constraints/MaximumPosition');
    const rawTarget = v('robot/turret/Values/target');
    const target = Math.min(maxPos, Math.max(minPos, rawTarget));
    const maxVel = Math.min(TURRET_SPEED_LIMIT, v('robot/turret/Controls/position/Config/Constraints/MaximumVelocity'));
    const maxAcc = v('robot/turret/Controls/position/Config/Constraints/MaximumAcceleration');

    // Reference follows the target with a velocity and acceleration limit (trapezoidal-style).
    const refError = target - this.turretReference;
    const desiredVel = Math.max(-maxVel, Math.min(maxVel, refError / 0.25));
    const dv = Math.max(-maxAcc * dt, Math.min(maxAcc * dt, desiredVel - this.turretVelocity));
    this.turretVelocity += dv;
    this.turretReference += this.turretVelocity * dt;
    this.turretPosition += (this.turretReference - this.turretPosition) * Math.min(1, dt * 12);

    const error = target - this.turretPosition;
    // Controls/position/State/Feedback* are the measured traces; the reference stays internal to the model.
    this.values.set('robot/turret/Controls/position/State/FeedbackPosition', this.turretPosition + this.noise());
    this.values.set('robot/turret/Controls/position/State/FeedbackVelocity', this.turretVelocity);
    this.values.set('robot/turret/Devices/encoder/State/Position', this.turretPosition + this.noise());

    const current = 2 + Math.abs(this.turretVelocity) * 0.15;
    this.values.set('robot/turret/Devices/motor/State/SupplyCurrentAmps', current);
    this.values.set('robot/turret/Devices/motor/State/StatorCurrentAmps', current * 1.2);
    this.values.set('robot/turret/Devices/motor/State/AppliedVoltage', 12 * Math.min(1, 0.2 + Math.abs(this.turretVelocity) / TURRET_SPEED_LIMIT));
    this.values.set('robot/turret/Devices/motor/State/VelocityRotationsPerSecond', this.turretVelocity / 360);

    let reason = 'At setpoint';
    if (homing) reason = 'Homing: target forced to 0';
    else if (rawTarget !== target) reason = 'Guard: at range edge';
    else if (Math.abs(error) >= 0.5) reason = `Moving: error ${error.toFixed(1)} deg`;
    this.values.set('robot/turret/State/Reason', reason);
    this.values.set('robot/turret/State/ActiveAction', homing ? 'home' : '');
    this.values.set('robot/turret/State/ActionRunning', homing);
    this.values.set('robot/turret/State/ActionComplete', !homing && Math.abs(error) < 0.5);

    // Hood follows its own target.
    this.hoodPosition += (v('robot/turret/hood/Values/target') - this.hoodPosition) * Math.min(1, dt * 6);
    this.values.set('robot/turret/hood/Devices/encoder/State/Position', this.hoodPosition + this.noise());
    const hoodSettled = Math.abs(v('robot/turret/hood/Values/target') - this.hoodPosition) < 0.5;
    this.values.set('robot/turret/hood/State/Reason', hoodSettled ? 'At setpoint' : 'Moving');
  }

  private stepShooter(dt: number): void {
    const v = (p: string) => this.values.get(p) as number;
    const target = v('robot/shooter/Values/targetVelocity');
    const maxAcc = v('robot/shooter/Controls/velocity/Config/Constraints/MaximumAcceleration');
    const step = Math.max(-maxAcc * dt, Math.min(maxAcc * dt, target - this.shooterReference));
    this.shooterReference += step;
    this.shooterVelocity += (this.shooterReference - this.shooterVelocity) * Math.min(1, dt / 0.4);

    const current = 8 + Math.abs(this.shooterVelocity) * 0.02;
    this.values.set('robot/shooter/Controls/velocity/State/FeedbackVelocity', this.shooterVelocity + this.noise() * 40);
    this.values.set('robot/shooter/Values/measuredVelocityRps', this.shooterVelocity / 60);
    this.values.set('robot/shooter/Devices/motor/State/SupplyCurrentAmps', current);
    this.values.set('robot/shooter/Devices/motor/State/StatorCurrentAmps', current * 1.2);
    this.values.set('robot/shooter/Devices/motor/State/VelocityRotationsPerSecond', this.shooterVelocity / 60);

    const ready = target > 0 && Math.abs(this.shooterVelocity - target) < 50;
    const spinning = target > 0 && !ready;
    this.values.set('robot/shooter/State/ActiveAction', spinning ? 'normalPower' : '');
    this.values.set('robot/shooter/State/ActionRunning', spinning);
    this.values.set('robot/shooter/State/ActionComplete', ready);
    this.values.set(
      'robot/shooter/State/Reason',
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
    } else if (path === 'robot/shooter/Actions/normalPower') {
      this.values.set('robot/shooter/Values/targetVelocity', 3000);
    } else if (path === 'robot/shooter/Actions/idle' || path === 'robot/shooter/Actions/stop') {
      this.values.set('robot/shooter/Values/targetVelocity', 0);
    }
    return { accepted: true, reason: null };
  }

  /** Cancels a running action. Turret home is the only cancellable one here. */
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
