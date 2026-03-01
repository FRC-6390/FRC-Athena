package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import org.junit.jupiter.api.Test;

final class RobotLocalizationHeadingAxisResetTest {

    @Test
    void resetPosePreservesDriverAndDriftOffsetsFromField() {
        TestImu imu = new TestImu();
        imu.setVirtualAxis("field", Rotation2d.fromDegrees(10.0));
        imu.setVirtualAxis("driver", Rotation2d.fromDegrees(30.0));
        imu.setVirtualAxis("drift", Rotation2d.fromDegrees(20.0));

        RobotLocalization<DifferentialDriveWheelPositions> localization = new RobotLocalization<>(
                new DifferentialTestEstimatorFactory(0.62),
                new RobotLocalizationConfig(),
                new RobotSpeeds(4.5, Math.PI),
                imu,
                () -> new DifferentialDriveWheelPositions(0.0, 0.0));

        Rotation2d baselineField = imu.getVirtualAxis("field");
        Rotation2d baselineDriver = imu.getVirtualAxis("driver");
        Rotation2d baselineDrift = imu.getVirtualAxis("drift");
        Rotation2d driverOffset = baselineDriver.minus(baselineField);
        Rotation2d driftOffset = baselineDrift.minus(baselineField);

        localization.resetPose("field", new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)));

        assertEquals(90.0, imu.getVirtualAxis("field").getDegrees(), 1e-9);
        assertEquals(90.0 + driverOffset.getDegrees(), imu.getVirtualAxis("driver").getDegrees(), 1e-9);
        assertEquals(90.0 + driftOffset.getDegrees(), imu.getVirtualAxis("drift").getDegrees(), 1e-9);
    }

    private static final class DifferentialTestEstimatorFactory
            implements PoseEstimatorFactory<DifferentialDriveWheelPositions> {
        private final DifferentialDriveKinematics kinematics;

        private DifferentialTestEstimatorFactory(double trackWidthMeters) {
            this.kinematics = new DifferentialDriveKinematics(trackWidthMeters);
        }

        @Override
        public PoseEstimator<DifferentialDriveWheelPositions> create2d(
                Pose2d startPose,
                Matrix<N3, N1> stateStdDevs,
                Matrix<N3, N1> visionStdDevs) {
            return new DifferentialDrivePoseEstimator(
                    kinematics,
                    startPose.getRotation(),
                    0.0,
                    0.0,
                    startPose,
                    stateStdDevs,
                    visionStdDevs);
        }

        @Override
        public PoseEstimator3d<DifferentialDriveWheelPositions> create3d(
                Pose3d startPose,
                Matrix<N4, N1> stateStdDevs,
                Matrix<N4, N1> visionStdDevs) {
            throw new UnsupportedOperationException("2D estimator only for this test.");
        }
    }

    private static final class TestImu implements Imu {
        private static final ImuType TYPE = () -> "test";
        private final ImuConfig config = ImuConfig.create(TYPE);
        private final Map<String, Rotation2d> virtualAxes = new HashMap<>();
        private Rotation2d yaw = new Rotation2d();

        @Override
        public Rotation2d getRoll() {
            return new Rotation2d();
        }

        @Override
        public Rotation2d getPitch() {
            return new Rotation2d();
        }

        @Override
        public Rotation2d getYaw() {
            return yaw;
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public void setYaw(Rotation2d yaw) {
            this.yaw = yaw == null ? new Rotation2d() : yaw;
            setVirtualAxis("driver", this.yaw);
        }

        @Override
        public void addVirtualAxis(String name, Supplier<Rotation2d> supplier) {
            virtualAxes.put(name, supplier == null ? new Rotation2d() : supplier.get());
        }

        @Override
        public Rotation2d getVirtualAxis(String name) {
            return virtualAxes.getOrDefault(name, new Rotation2d());
        }

        @Override
        public void setVirtualAxis(String name, Rotation2d value) {
            virtualAxes.put(name, value == null ? new Rotation2d() : value);
        }

        @Override
        public ImuConfig getConfig() {
            return config;
        }

        @Override
        public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
            return node;
        }
    }
}
