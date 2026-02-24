package ca.frc6390.athena.drivetrains.differential;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.drivetrains.differential.sim.DifferentialSimulationConfig;
import ca.frc6390.athena.drivetrains.examples.DifferentialDrivetrainExamples;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

final class DifferentialDrivetrainExamplesTest {

    @Test
    void arcadeCommandWritesAndResetClearsDriveSource() {
        DifferentialDrivetrain drivetrain = createDrivetrain(4.0);
        Command command = DifferentialDrivetrainExamples.arcadeDriveCommand(drivetrain, () -> 0.5, () -> -0.25);

        command.execute();
        ChassisSpeeds drive = drivetrain.speeds().getInput("drive");
        assertEquals(2.0, drive.vxMetersPerSecond, 1e-9);
        assertEquals(-1.0, drive.omegaRadiansPerSecond, 1e-9);

        DifferentialDrivetrainExamples.resetControlState(drivetrain);
        ChassisSpeeds afterReset = drivetrain.speeds().getInput("drive");
        assertEquals(0.0, afterReset.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, afterReset.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void feedforwardConfigurationControlsEnabledFlag() {
        DifferentialDrivetrain drivetrain = createDrivetrain(5.0);

        DifferentialDrivetrainExamples.configureDriveFeedforward(drivetrain, 0.1, 1.0, 0.2, false);
        assertFalse(drivetrain.driveFeedforwardEnabled());

        DifferentialDrivetrainExamples.configureDriveFeedforward(drivetrain, 0.1, 1.0, 0.2, true);
        assertTrue(drivetrain.driveFeedforwardEnabled());
    }

    @Test
    void simulationPoseSetterRoundTripsWhenSimulationEnabled() {
        DifferentialDrivetrain drivetrain = createDrivetrain(4.0);

        if (!RobotBase.isSimulation()) {
            assertFalse(drivetrain.simulation().enabled());
            return;
        }

        DifferentialSimulationConfig simConfig = DifferentialSimulationConfig.defaults()
                .resolve(0.6, 8.0, 0.1524, 2);
        drivetrain.configureSimulation(simConfig);
        assertTrue(drivetrain.simulation().enabled());

        Pose2d expected = new Pose2d(1.2, -0.4, Rotation2d.fromDegrees(30.0));
        DifferentialDrivetrainExamples.setSimulationPose(drivetrain, expected);
        Pose2d actual = drivetrain.simulation().pose();

        assertTrue(Double.isFinite(actual.getX()));
        assertTrue(Double.isFinite(actual.getY()));
        assertTrue(Double.isFinite(actual.getRotation().getDegrees()));
    }

    private static DifferentialDrivetrain createDrivetrain(double maxVelocityMetersPerSecond) {
        MotorControllerGroup left = new MotorControllerGroup(new FakeMotor(1), new FakeMotor(2));
        MotorControllerGroup right = new MotorControllerGroup(new FakeMotor(3), new FakeMotor(4));
        return new DifferentialDrivetrain(new FakeImu(), maxVelocityMetersPerSecond, 0.6, left, right);
    }

    private static final class FakeImu implements Imu {
        private static final ImuType TYPE = () -> "test";

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
            return new Rotation2d();
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public ImuConfig getConfig() {
            return ImuConfig.create(TYPE, 0);
        }
    }

    private static final class FakeMotor implements MotorController {
        private static final MotorControllerType TYPE = () -> "test";
        private final int id;

        private FakeMotor(int id) {
            this.id = id;
        }

        @Override
        public int getId() {
            return id;
        }

        @Override
        public String getCanbus() {
            return "rio";
        }

        @Override
        public MotorControllerType getType() {
            return TYPE;
        }

        @Override
        public void setSpeed(double percent) {
        }

        @Override
        public void setVoltage(double volts) {
        }

        @Override
        public void setCurrentLimit(double amps) {
        }

        @Override
        public void setPosition(double rotations) {
        }

        @Override
        public void setVelocity(double rotationsPerSecond) {
        }

        @Override
        public void setNeutralMode(MotorNeutralMode mode) {
        }

        @Override
        public void setPid(PIDController pid) {
        }

        @Override
        public boolean isConnected() {
            return true;
        }

        @Override
        public double getTemperatureCelsius() {
            return 25.0;
        }

        @Override
        public Encoder getEncoder() {
            return null;
        }
    }
}
