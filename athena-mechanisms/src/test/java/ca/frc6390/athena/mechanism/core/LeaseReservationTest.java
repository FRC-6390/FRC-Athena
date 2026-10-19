package ca.frc6390.athena.mechanism.core;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Regression tests for dynamic lease reservations and lease cancellation scope. */
class LeaseReservationTest {
    private static final MotorDevice DRIVE = MotorDevice.of(MotorKinds.KRAKEN_X60, 41);
    private static final MotorDevice TURRET = MotorDevice.of(MotorKinds.KRAKEN_X60, 42);
    private static final MotorDevice INTAKE = MotorDevice.of(MotorKinds.KRAKEN_X44, 43);
    private static final MotorDevice ARM = MotorDevice.of(MotorKinds.KRAKEN_X60, 44);

    @Test
    void completedPathReleasesDrivetrainSoOlderPooledDriveAppliesFaceHub() {
        boolean[] enabled = {true};
        RecordingContext hardware = new RecordingContext();
        DriveRobot robot = new DriveRobot(enabled);
        PathAction path = new PathAction("test", "ToHub");
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        scheduler.path(path, new OwnedPath(0.1, List.of(robot.pooledDrive), robot.pooledDrive));
        scheduler.declarationOwner(path, robot);
        Action auto = Actions.sequence().run(path).forTime(Seconds.of(10.0), robot.faceHub);

        scheduler.teleopPeriodic(0.00, 0.02);
        scheduler.request(auto);
        for (int cycle = 1; cycle <= 4; cycle++) scheduler.autoPeriodic(cycle * 0.02, 0.02);
        assertFalse(robot.angular.isActive(), "path is still running");

        // Path complete: only faceHub (angular sink) remains. The older whileActive pooledDrive
        // lease must now own the drivetrain and turn the robot.
        for (int cycle = 5; cycle <= 10; cycle++) scheduler.autoPeriodic(cycle * 0.02, 0.02);
        assertTrue(robot.angular.isActive());
        assertEquals(0.5, robot.angular.radiansPerSecond(), 1.0e-9);
        assertEquals(0.5, hardware.motor(DRIVE).percent, 1.0e-9);
    }

    @Test
    void homingRunsDuringAutoUntilTheTurretStageIsImminent() {
        boolean[] homed = {false};
        RecordingContext hardware = new RecordingContext();
        TurretRobot robot = new TurretRobot(homed);
        PathAction path = new PathAction("test", "Leave");
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        scheduler.path(path, new OwnedPath(0.2, List.of(DRIVE), DRIVE.percent(0.7)));
        scheduler.declarationOwner(path, robot);
        Action auto = Actions.sequence()
                .run(path)
                .forTime(Seconds.of(0.1), INTAKE.percent(0.6))
                .forTime(Seconds.of(1.0), TURRET.percent(0.4));

        // Homing starts at autonomous init; the auto is requested on the first periodic cycle.
        scheduler.periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, true, false),
                new EventContext(0.0, 0.02, LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, false));
        scheduler.request(auto);
        scheduler.autoPeriodic(0.02, 0.02);
        scheduler.autoPeriodic(0.04, 0.02);
        assertEquals(0.7, hardware.motor(DRIVE).percent, 1.0e-9);
        assertEquals(-0.2, hardware.motor(TURRET).percent, 1.0e-9, "homing must not be starved by a distant stage");

        // The intake stage is followed directly by the turret stage, so the auto now reserves it.
        double now = 0.06;
        for (; now < 0.25; now += 0.02) scheduler.autoPeriodic(now, 0.02);
        assertEquals(0.6, hardware.motor(INTAKE).percent, 1.0e-9);
        assertEquals(0.0, hardware.motor(TURRET).percent, 1.0e-9);

        homed[0] = true;
        for (int cycle = 0; cycle < 6; cycle++, now += 0.02) scheduler.autoPeriodic(now, 0.02);
        assertEquals(0.4, hardware.motor(TURRET).percent, 1.0e-9);
    }

    @Test
    void olderLeaseNeverFillsTheGapBetweenConsecutiveStagesUsingTheSameMotor() {
        boolean[] held = {true};
        boolean[] advance = {false};
        RecordingContext hardware = new RecordingContext();
        ArmRobot robot = new ArmRobot(held);
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        Action auto = Actions.sequence()
                .until(() -> advance[0], ARM.percent(0.3))
                .forTime(Seconds.of(1.0), ARM.percent(0.5));

        scheduler.teleopPeriodic(0.0, 0.02);
        assertEquals(-0.9, hardware.motor(ARM).percent, 1.0e-9);
        scheduler.request(auto);
        hardware.motor(ARM).history.clear();
        for (int cycle = 1; cycle < 12; cycle++) {
            advance[0] = cycle >= 4;
            scheduler.teleopPeriodic(cycle * 0.02, 0.02);
        }

        assertFalse(hardware.motor(ARM).history.isEmpty());
        assertFalse(hardware.motor(ARM).history.contains(-0.9), "older lease glitched onto the arm");
        assertTrue(hardware.motor(ARM).history.contains(0.5));
    }

    @Test
    void cancellingAnAutoKeepsIndependentlyRequestedNestedActionsRunning() {
        RecordingContext hardware = new RecordingContext();
        ArmRobot robot = new ArmRobot(new boolean[] {false});
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        Action auto = Actions.sequence().forTime(Seconds.of(0.04), robot.collect).waitSeconds(Seconds.of(10.0));

        scheduler.request(auto);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.request(robot.collect);
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.cancel(auto);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertTrue(scheduler.isRunning(robot.collect));
        assertEquals(0.8, hardware.motor(INTAKE).percent, 1.0e-9);
    }

    @Test
    void cancellingAnAutoCancelsLeasesItRequestedWhileRunning() {
        RecordingContext hardware = new RecordingContext();
        ArmRobot robot = new ArmRobot(new boolean[] {false});
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        Action auto = Actions.sequence()
                .doOnce(() -> scheduler.request(robot.collect))
                .waitSeconds(Seconds.of(10.0));

        scheduler.request(auto);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.teleopPeriodic(0.02, 0.02);
        assertTrue(scheduler.isRunning(robot.collect));

        scheduler.cancel(auto);
        scheduler.teleopPeriodic(0.04, 0.02);
        assertFalse(scheduler.isRunning(robot.collect));
        assertEquals(0.0, hardware.motor(INTAKE).percent, 1.0e-9);
    }

    /**
     * A restrictive root reserves every path it can reach. A concurrent request for the drive motor must
     * be held while a drive path runs, whether the path is a plain step or sits under then(...) and
     * until(...).then(...).
     */
    @Test
    void pathInAStepIsReservedForConcurrentRequests() {
        boolean[] done = {false};
        Action auto = Actions.sequence()
                .run(intakePath("IntakeA"))
                .run(drivePath("DriveStep"))
                .timeout(Seconds.of(30.0));
        assertConcurrentDriveHeldWhileDrivePathRuns(auto, 0.5, () -> done[0] = true);
    }

    @Test
    void pathUnderThenAndUntilIsReservedForConcurrentRequests() {
        boolean[] done = {false};
        Action auto = Actions.sequence()
                .run(intakePath("IntakeA"))
                .then(intakePath("IntakeB"))
                .then(drivePath("DriveThen").until(() -> done[0]).then(intakePath("IntakeY")))
                .timeout(Seconds.of(30.0));
        assertConcurrentDriveHeldWhileDrivePathRuns(auto, 1000.0, () -> done[0] = true);
    }

    @Test
    void pathUnderTwoThenStagesIsReservedForConcurrentRequests() {
        boolean[] done = {false};
        Action auto = Actions.sequence()
                .run(intakePath("IntakeA"))
                .then(intakePath("IntakeB"))
                .then(intakePath("IntakeC"))
                .then(drivePath("DriveLate").until(() -> done[0]))
                .timeout(Seconds.of(30.0));
        assertConcurrentDriveHeldWhileDrivePathRuns(auto, 1000.0, () -> done[0] = true);
    }

    private static PathAction intakePath(String name) {
        return new PathAction("test", name);
    }

    private static PathAction drivePath(String name) {
        return new PathAction("test", name);
    }

    private static void assertConcurrentDriveHeldWhileDrivePathRuns(
            Action auto, double driveSeconds, Runnable releaseDrive) {
        RecordingContext hardware = new RecordingContext();
        PlainRobot robot = new PlainRobot();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        for (PathAction path : PathIntrospector.inspect(auto)) {
            if (path.name().startsWith("Drive")) {
                scheduler.path(path, new OwnedPath(driveSeconds, List.of(DRIVE), DRIVE.percent(0.7)));
            } else {
                scheduler.path(path, new OwnedPath(0.1, List.of(INTAKE), INTAKE.percent(0.5)));
            }
            scheduler.declarationOwner(path, robot);
        }
        Action concurrent = DRIVE.percent(-0.3);

        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.request(auto.interruptible(Interrupt.AFTER_COMPLETE));
        double now = 0.02;
        for (int cycle = 0; cycle < 100 && hardware.motor(DRIVE).percent != 0.7; cycle++, now += 0.02) {
            scheduler.autoPeriodic(now, 0.02);
        }
        assertEquals(0.7, hardware.motor(DRIVE).percent, 1.0e-9, "drive path must be running");

        scheduler.request(concurrent);
        for (int cycle = 0; cycle < 5; cycle++, now += 0.02) scheduler.autoPeriodic(now, 0.02);
        assertFalse(hardware.motor(DRIVE).history.contains(-0.3),
                "a concurrent drive request ran over a drive path reached through then(...)");

        releaseDrive.run();
        for (int cycle = 0; cycle < 200 && !hardware.motor(DRIVE).history.contains(-0.3); cycle++, now += 0.02) {
            scheduler.autoPeriodic(now, 0.02);
        }
        assertTrue(hardware.motor(DRIVE).history.contains(-0.3),
                "the held drive request must run once the auto completes");
    }

    /** Declares the motors so requests against them resolve to this mechanism. */
    private static final class PlainRobot implements Mechanism {
        @SuppressWarnings("unused")
        private final MotorDevice drive = DRIVE;
        @SuppressWarnings("unused")
        private final MotorDevice intake = INTAKE;
    }

    private static final class DriveRobot implements Mechanism {
        private final RobotVelocityPool pool = new RobotVelocityPool();
        private final RobotVelocityPool.AngularChannel angular = pool.angularChannel();
        private final ControlBinding<?> heading = Controls.position(angular)
                .feedback(() -> 0.0)
                .pid(1.0, 0.0, 0.0)
                .constraint(Constraints.clamp(0.5));
        private final Action pooledDrive = DRIVE.percent(() -> pool.robotRelative(0.0).angularRadiansPerSecond());
        private final Action faceHub = heading.position(2.0);
        @SuppressWarnings("unused")
        private final MotorDevice driveMotor = DRIVE;
        @SuppressWarnings("unused")
        private final HookBinding drive;

        private DriveRobot(boolean[] enabled) {
            drive = Events.when(() -> enabled[0]).active().whileActive(pooledDrive);
        }
    }

    private static final class TurretRobot implements Mechanism {
        private final Action home;
        @SuppressWarnings("unused")
        private final MotorDevice drive = DRIVE;
        @SuppressWarnings("unused")
        private final MotorDevice intake = INTAKE;
        @SuppressWarnings("unused")
        private final MotorDevice turret = TURRET;
        @SuppressWarnings("unused")
        private final HookBinding homing;

        private TurretRobot(boolean[] homed) {
            home = Actions.until(() -> homed[0], TURRET.percent(-0.2));
            homing = Events.when(() -> !homed[0]).rising().onStart(home);
        }
    }

    private static final class ArmRobot implements Mechanism {
        private final Action hold = ARM.percent(-0.9);
        private final Action collect = INTAKE.percent(0.8);
        @SuppressWarnings("unused")
        private final MotorDevice arm = ARM;
        @SuppressWarnings("unused")
        private final MotorDevice intake = INTAKE;
        @SuppressWarnings("unused")
        private final HookBinding holdBinding;

        private ArmRobot(boolean[] held) {
            holdBinding = Events.when(() -> held[0]).active().whileActive(hold);
        }
    }

    /** Minimal provider path: fixed duration, declared ownership, constant drive output. */
    private record OwnedPath(double seconds, List<?> ownership, Action drive) implements PathRuntime {
        @Override public Action output(PathAction path, MechanismContext context) { return drive; }
        @Override public List<?> ownership(PathAction path) { return ownership; }
        @Override public boolean isFinished(PathAction path, MechanismContext context) {
            return context.timeInStateSeconds() + 1.0e-9 >= seconds;
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new HashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private final List<Double> history = new ArrayList<>();
        private double percent = Double.NaN;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
            history.add(percent);
        }

        @Override
        public void stop() {
            percent = 0.0;
            history.add(0.0);
        }
    }
}
