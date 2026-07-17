package ca.frc6390.athena.drivetrain.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import ca.frc6390.athena.runtime.control.VelocityFrame;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class SwervePathFollowerTest {
    @Test
    void kinematicsOwnedFollowerCombinesFeedforwardPoseErrorAndWrappedHeading() {
        AtomicReference<Pose2d> pose = new AtomicReference<>(
                new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(179.0)));
        PidGains translation = PidGains.of(2.0, 0.0, 0.0);
        PidGains heading = PidGains.of(3.0, 0.0, 0.0);
        SwervePathFollower follower = kinematics().follow(
                FollowerBackend.CHOREO,
                pose::get,
                target -> Actions.doOnce(() -> pose.set(target)),
                translation,
                heading);

        RobotVelocity actual = follower.calculateVelocity(new SwervePathSample(
                new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(-179.0)),
                RobotVelocity.field(0.5, -0.25, 0.1)), 0.02);
        RobotVelocity expected = RobotVelocity.field(
                2.5, 3.75, 0.1 + 3.0 * Math.toRadians(2.0))
                .fieldToRobot(Math.toRadians(179.0));

        assertEquals(expected.xMetersPerSecond(), actual.xMetersPerSecond(), 1.0e-9);
        assertEquals(expected.yMetersPerSecond(), actual.yMetersPerSecond(), 1.0e-9);
        assertEquals(expected.angularRadiansPerSecond(), actual.angularRadiansPerSecond(), 1.0e-9);
        assertEquals(FollowerBackend.CHOREO, follower.backend());
        assertSame(translation, follower.translationGains());
        assertSame(heading, follower.headingGains());
    }

    @Test
    void liveGainOverridesImmediatelyChangeFollowerAndResetActionStaysAnAction() {
        AtomicReference<Pose2d> pose = new AtomicReference<>(new Pose2d());
        PidGains translation = PidGains.of(1.0, 0.0, 0.0);
        SwervePathFollower follower = kinematics().follow(
                FollowerBackend.CHOREO,
                pose::get,
                target -> Actions.doOnce(() -> pose.set(target)),
                translation,
                PidGains.of(0.0, 0.0, 0.0));
        SwervePathSample sample = new SwervePathSample(
                new Pose2d(2.0, 0.0, new Rotation2d()),
                RobotVelocity.zero(VelocityFrame.FIELD));

        assertEquals(2.0, follower.calculateVelocity(sample, 0.02).xMetersPerSecond(), 1.0e-9);
        follower.telemetry().get("translation/p").set(3.0);
        assertEquals(6.0, follower.calculateVelocity(sample, 0.02).xMetersPerSecond(), 1.0e-9);
        follower.telemetry().get("translation/disabled").set(true);
        assertEquals(0.0, follower.calculateVelocity(sample, 0.02).xMetersPerSecond(), 1.0e-9);

        Pose2d resetTarget = new Pose2d(4.0, 5.0, Rotation2d.fromDegrees(20.0));
        Action reset = follower.resetPose(resetTarget);
        ((Actions.DoOnce) reset).action().run();
        assertEquals(resetTarget, pose.get());
    }

    @Test
    void pooledFollowerPublishesVelocityReturnsSharedDriveAndClearsOnStop() {
        SwerveKinematics kinematics = kinematics();
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.Channel auto = pool.channel();
        Action sharedDrive = kinematics.drive(pool, () -> 0.0);
        SwervePathFollower follower = kinematics.follow(
                FollowerBackend.CHOREO,
                Pose2d::new,
                ignored -> Actions.doOnce(() -> { }),
                PidGains.of(0.0, 0.0, 0.0),
                PidGains.of(0.0, 0.0, 0.0))
                .pooled(auto, sharedDrive);

        Action output = follower.follow(new SwervePathSample(
                new Pose2d(),
                RobotVelocity.field(1.5, -0.25, 0.4)), 0.02);

        assertSame(sharedDrive, output);
        assertEquals(RobotVelocity.robot(1.5, -0.25, 0.4), auto.velocity());
        follower.stop();
        assertFalse(auto.isActive());
    }

    private static SwerveKinematics kinematics() {
        return SwerveKinematics.rectangular(
                0.55, 0.55, 4.0,
                module(1, 5, 9), module(2, 6, 10), module(3, 7, 11), module(4, 8, 12));
    }

    private static SwerveModule module(int driveId, int steerId, int encoderId) {
        return new SwerveModules.SDS.MK5N.R3()
                .drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId))
                .steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId))
                .angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, encoderId))
                .driveMaxSpeedMetersPerSecond(4.0)
                .steerPid(12.0, 0.0, 0.0);
    }
}
