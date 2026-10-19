package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.DriverOrientation;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleModel;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Runs the driver orientation through a real RobotRuntime: teleop-start hook, reset and restore actions, and the drive overload. */
class DriverOrientationRuntimeTest {
    private static final double PI = Math.PI;
    private static final double EPS = 1.0e-9;
    private static final double DT = 0.02;

    @Test
    void teleopEnableAssumesDriverForwardOnRedWhenThePoseIsUnseeded() {
        Rig rig = new Rig(Optional.of(DriverOrientation.Alliance.RED), 0.9, false);

        rig.teleopInit();

        assertEquals(1, rig.resets.size());
        assertEquals(3.0, rig.pose.xMeters(), EPS, "translation is kept");
        assertEquals(PI, rig.pose.headingRadians(), EPS, "red assumes facing away from its own wall");
        assertEquals(0.0, rig.drive.orientation.driverHeadingRadians(), EPS);
    }

    @Test
    void teleopEnableAssumesBlueForwardWhenUnknownAllianceAndUnseeded() {
        Rig rig = new Rig(Optional.empty(), 0.4, false);

        rig.teleopInit();

        assertEquals(0.0, rig.pose.headingRadians(), EPS);
        assertEquals(0.0, rig.drive.orientation.driverHeadingRadians(), EPS);
    }

    @Test
    void seededPoseIsLeftAloneAtTeleopEnable() {
        Rig rig = new Rig(Optional.of(DriverOrientation.Alliance.RED), 0.9, true);

        rig.teleopInit();

        assertTrue(rig.resets.isEmpty());
        assertEquals(0.9, rig.pose.headingRadians(), EPS);
    }

    @Test
    void driverResetActionMakesTheCurrentFacingForwardThroughTheRuntime() {
        Rig rig = new Rig(Optional.of(DriverOrientation.Alliance.BLUE), 0.7, true);

        rig.run(rig.drive.resetDriverHeading);

        assertEquals(0.0, rig.drive.orientation.driverHeadingRadians(), EPS);
        assertEquals(0, rig.resets.size(), "the reset never touches the pose");
        rig.pose = new PoseSnapshot(3.0, 4.0, 1.2);
        assertEquals(0.5, rig.drive.orientation.driverHeadingRadians(), EPS);
    }

    @Test
    void restoreActionClearsTheDriverResetThroughTheRuntime() {
        Rig rig = new Rig(Optional.of(DriverOrientation.Alliance.RED), 0.7, true);
        rig.run(rig.drive.resetDriverHeading);

        rig.run(rig.drive.restoreForward);

        assertEquals(PI, rig.drive.orientation.forwardRadians(), EPS);
        assertEquals(0.7 - PI, rig.drive.orientation.driverHeadingRadians(), EPS);
    }

    @Test
    void midAutoPoseResetDoesNotMoveDriverForward() {
        Rig rig = new Rig(Optional.of(DriverOrientation.Alliance.RED), 0.3, true);
        double forward = rig.drive.orientation.forwardRadians();

        rig.pose = new PoseSnapshot(8.0, 2.0, -2.0);

        assertEquals(forward, rig.drive.orientation.forwardRadians(), EPS);
        assertEquals(wrap(-2.0 - forward), rig.drive.orientation.driverHeadingRadians(), EPS);
    }

    @Test
    void driveOverloadSendsDriverForwardStickToTheFieldDirectionOfTheDriverStation() {
        // Robot facing field +X at heading 0: on blue that is driver forward, on red it is driver backward.
        Rig blue = new Rig(Optional.of(DriverOrientation.Alliance.BLUE), 0.0, true);
        Rig red = new Rig(Optional.of(DriverOrientation.Alliance.RED), 0.0, true);

        double blueDrive = driveForwardCommand(blue);
        double redDrive = driveForwardCommand(red);

        assertTrue(blueDrive > 0.0, "blue forward drives the wheels forward: " + blueDrive);
        assertEquals(blueDrive, -redDrive, EPS, "red forward is the reverse of blue forward for the same robot");
    }

    private static double driveForwardCommand(Rig rig) {
        rig.drive.velocity.channel().set(RobotVelocity.field(1.0, 0.0, 0.0));
        rig.run(rig.drive.pooledDrive);
        return rig.simulation.motor(rig.drive.frontLeftDrive).commandValue();
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static final class Rig {
        final List<PoseSnapshot> resets = new ArrayList<>();
        final SimulationSession simulation = SimulationSession.create();
        final RobotRuntime runtime;
        final DriveMechanism drive;
        Optional<DriverOrientation.Alliance> alliance;
        PoseSnapshot pose;
        double now = 0.0;

        Rig(Optional<DriverOrientation.Alliance> alliance, double heading, boolean seeded) {
            this.alliance = alliance;
            this.pose = new PoseSnapshot(3.0, 4.0, heading);
            this.drive = new DriveMechanism(this, seeded);
            this.runtime = RobotRuntime.simulated(simulation).register(drive);
        }

        void teleopInit() {
            now += DT;
            runtime.periodic(
                    new MechanismContext(now, 0.0, DT, true, false, false),
                    new EventContext(now, DT, LifecycleMode.TELEOP, LifecyclePhase.INIT, true, false));
        }

        void run(Action action) {
            runtime.request(action);
            now += DT;
            runtime.teleopPeriodic(now, DT);
        }
    }

    private static final class DriveMechanism implements Mechanism {
        final DriverOrientation orientation;
        final RobotVelocityPool velocity = new RobotVelocityPool();
        final Action resetDriverHeading;
        final Action restoreForward;
        final Action pooledDrive;
        final MotorDevice frontLeftDrive;
        private final SwerveModule frontLeft = module(1, 11, 21);
        private final SwerveModule frontRight = module(2, 12, 22);
        private final SwerveModule backLeft = module(3, 13, 23);
        private final SwerveModule backRight = module(4, 14, 24);
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.5, 0.5, 4.0, frontLeft, frontRight, backLeft, backRight);

        DriveMechanism(Rig rig, boolean seeded) {
            this.frontLeftDrive = frontLeft.drive.get();
            this.orientation = DriverOrientation.of(
                    () -> rig.pose,
                    () -> rig.alliance,
                    () -> seeded || !rig.resets.isEmpty(),
                    next -> {
                        rig.resets.add(next);
                        return ActionBinding.run(() -> rig.pose = next);
                    });
            this.resetDriverHeading = orientation.resetHeading();
            this.restoreForward = orientation.restoreAllianceForward();
            this.pooledDrive = kinematics.drive(velocity, orientation);
        }

        private static SwerveModule module(int driveId, int steerId, int encoderId) {
            TestModule module = new TestModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, encoderId).units(EncoderUnit.ROTATIONS));
            return module.driveMaxSpeedMetersPerSecond(4.0).steerPid(12.0, 0.0, 0.0);
        }
    }

    private static final class TestModule extends SwerveModule {
        private TestModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }
}
