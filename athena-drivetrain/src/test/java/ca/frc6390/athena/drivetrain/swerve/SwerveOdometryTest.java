package ca.frc6390.athena.drivetrain.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class SwerveOdometryTest {
    @Test
    void integratesMeasuredWheelTravelAndImuHeading() {
        TestRig rig = new TestRig();
        rig.odometry.refresh(rig.context, 0.0, 0.02);

        rig.drivePositions.replaceAll((id, ignored) -> 2.0);
        rig.odometry.refresh(rig.context, 1.0, 1.0);
        assertPose(rig.odometry.pose(), 1.0, 0.0, 0.0);

        rig.moduleAngles.replaceAll((id, ignored) -> 0.25);
        rig.drivePositions.replaceAll((id, position) -> position + 2.0);
        rig.odometry.refresh(rig.context, 2.0, 1.0);
        assertPose(rig.odometry.pose(), 1.0, 1.0, 0.0);

        rig.imu.yawDegrees = 90.0;
        rig.odometry.refresh(rig.context, 3.0, 1.0);
        assertPose(rig.odometry.pose(), 1.0, 1.0, Math.PI / 2.0);
    }

    @Test
    void resetKeepsRequestedFieldPoseAcrossTheNextHardwareRefresh() {
        TestRig rig = new TestRig();
        rig.odometry.refresh(rig.context, 0.0, 0.02);
        rig.odometry.reset(new PoseSnapshot(5.0, 6.0, Math.PI));

        rig.odometry.refresh(rig.context, 1.0, 0.02);

        assertPose(rig.odometry.pose(), 5.0, 6.0, Math.PI);
    }

    @Test
    void measuredVelocityMatchesKinematicsForMixedModuleStates() {
        TestRig rig = new TestRig();
        rig.odometry.refresh(rig.context, 0.0, 0.02);

        rig.drivePositions.put(1, 1.0);
        rig.drivePositions.put(2, 2.0);
        rig.drivePositions.put(3, 3.0);
        rig.drivePositions.put(4, 4.0);
        rig.moduleAngles.put(21, 0.0);
        rig.moduleAngles.put(22, 0.125);
        rig.moduleAngles.put(23, 0.25);
        rig.moduleAngles.put(24, -0.125);
        rig.odometry.refresh(rig.context, 1.0, 1.0);

        List<SwerveOdometry.ModulePosition> positions = rig.odometry.modulePositions();
        RobotVelocity expected = rig.kinematics.velocity(List.of(
                target(positions.get(0)),
                target(positions.get(1)),
                target(positions.get(2)),
                target(positions.get(3))));
        assertEquals(expected.xMetersPerSecond(), rig.odometry.velocity().xMetersPerSecond(), 1.0e-9);
        assertEquals(expected.yMetersPerSecond(), rig.odometry.velocity().yMetersPerSecond(), 1.0e-9);
    }

    @Test
    void modulePositionSnapshotsAreCachedImmutableAndIndependentOfLaterRefreshes() {
        TestRig rig = new TestRig();
        assertEquals(List.of(), rig.odometry.modulePositions());
        rig.odometry.refresh(rig.context, 0.0, 0.02);

        List<SwerveOdometry.ModulePosition> initial = rig.odometry.modulePositions();
        assertSame(initial, rig.odometry.modulePositions());
        assertThrows(
                UnsupportedOperationException.class,
                () -> initial.add(new SwerveOdometry.ModulePosition(1.0, 0.0)));

        rig.drivePositions.replaceAll((id, ignored) -> 2.0);
        rig.moduleAngles.replaceAll((id, ignored) -> 0.25);
        rig.odometry.refresh(rig.context, 1.0, 1.0);

        List<SwerveOdometry.ModulePosition> updated = rig.odometry.modulePositions();
        assertNotSame(initial, updated);
        assertSame(updated, rig.odometry.modulePositions());
        for (SwerveOdometry.ModulePosition position : initial) {
            assertEquals(0.0, position.distanceMeters(), 1.0e-9);
            assertEquals(0.0, position.angleRotations(), 1.0e-9);
        }
        for (SwerveOdometry.ModulePosition position : updated) {
            assertEquals(1.0, position.distanceMeters(), 1.0e-9);
            assertEquals(0.25, position.angleRotations(), 1.0e-9);
        }
    }

    private static SwerveModuleTarget target(SwerveOdometry.ModulePosition position) {
        return new SwerveModuleTarget(position.distanceMeters(), position.angleRotations());
    }

    private static void assertPose(PoseSnapshot pose, double x, double y, double heading) {
        assertEquals(x, pose.xMeters(), 1.0e-9);
        assertEquals(y, pose.yMeters(), 1.0e-9);
        assertEquals(heading, pose.headingRadians(), 1.0e-9);
    }

    private static final class TestRig {
        private final Map<Integer, Double> drivePositions = initializedMap();
        private final Map<Integer, Double> moduleAngles = initializedMap();
        private final TestImu imu = new TestImu();
        private final SwerveModule frontLeft = module(1, 11, 21);
        private final SwerveModule frontRight = module(2, 12, 22);
        private final SwerveModule backLeft = module(3, 13, 23);
        private final SwerveModule backRight = module(4, 14, 24);
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.5, 0.5, 5.0, frontLeft, frontRight, backLeft, backRight);
        private final SwerveOdometry odometry = kinematics.odometry(imu);
        private final ActionContext context = new ActionContext() {
            @Override
            public EncoderHandle encoder(EncoderDevice device) {
                if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
                    return handle(device, drivePositions, integrated.motor().id());
                }
                return handle(device, moduleAngles, device.id());
            }
        };

        private static Map<Integer, Double> initializedMap() {
            Map<Integer, Double> values = new HashMap<>();
            for (int id = 1; id <= 24; id++) {
                values.put(id, 0.0);
            }
            return values;
        }

        private static EncoderHandle handle(EncoderDevice device, Map<Integer, Double> values, int id) {
            return new EncoderHandle() {
                @Override
                public EncoderDevice device() {
                    return device;
                }

                @Override
                public double positionRotations() {
                    return values.get(id);
                }

                @Override
                public double absolutePositionRotations() {
                    return values.get(id);
                }

                @Override
                public double velocityRotationsPerSecond() {
                    return 0.0;
                }
            };
        }

        private static SwerveModule module(int driveId, int steerId, int angleId) {
            TestModule module = new TestModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, angleId).units(EncoderUnit.ROTATIONS));
            return module;
        }
    }

    private static final class TestModule extends SwerveModule {
        private TestModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }

    private static final class TestImu implements ImuSource {
        private double yawDegrees;

        @Override public double yawDegrees() { return yawDegrees; }
        @Override public double pitchDegrees() { return 0.0; }
        @Override public double rollDegrees() { return 0.0; }
        @Override public double angleDegrees() { return yawDegrees; }
        @Override public double yawRateDegreesPerSecond() { return 0.0; }
        @Override public double linearAccelerationXG() { return 0.0; }
        @Override public double linearAccelerationYG() { return 0.0; }
        @Override public double linearAccelerationZG() { return 0.0; }
        @Override public void applyYaw(ActionContext context, double value) { yawDegrees = value; }
    }
}
