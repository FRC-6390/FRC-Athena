package ca.frc6390.athena.commands.movement;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.commands.examples.MovementCommandExamples;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class MovementCommandExamplesTest {

    @Test
    void createTranslationPidUsesLinearToleranceWithoutContinuousInput() {
        PIDController controller = MovementCommandExamples.createTranslationPid(0.8, 0.0, 0.0, 0.12);

        assertFalse(controller.isContinuousInputEnabled());
        controller.setSetpoint(0.0);
        controller.calculate(0.10);
        assertTrue(controller.atSetpoint());
        controller.calculate(0.20);
        assertFalse(controller.atSetpoint());
    }

    @Test
    void translateToPointWithPidAppliesConfiguredGainsToCommandOutput() {
        DifferentialDrivetrain drivetrain = createDrivetrain();
        PIDController controller = new PIDController(0.4, 0.0, 0.0);
        controller.setTolerance(0.02);

        TranslateToPoint command = MovementCommandExamples.translateToPointWithPid(drivetrain, 1.5, -0.5, controller);
        command.initialize();
        command.execute();

        ChassisSpeeds feedback = drivetrain.speeds().getInput(RobotSpeeds.FEEDBACK_SOURCE);
        assertNotEquals(0.0, feedback.vxMetersPerSecond, 1e-9);
        assertNotEquals(0.0, feedback.vyMetersPerSecond, 1e-9);
    }

    @Test
    void translateToPointFinishesAfterEstimatedTravelReachesTarget() {
        DifferentialDrivetrain drivetrain = createDrivetrain();
        PIDController controller = MovementCommandExamples.createTranslationPid(2.0, 0.0, 0.0, 0.02);
        TranslateToPoint command = MovementCommandExamples.translateToPointWithPid(drivetrain, 0.30, 0.0, controller);

        command.initialize();
        for (int i = 0; i < 300 && !command.isFinished(); i++) {
            command.execute();
        }
        assertTrue(command.isFinished());
    }

    private static DifferentialDrivetrain createDrivetrain() {
        MotorControllerGroup left = new MotorControllerGroup(new FakeMotor(1), new FakeMotor(2));
        MotorControllerGroup right = new MotorControllerGroup(new FakeMotor(3), new FakeMotor(4));
        return new DifferentialDrivetrain(new FakeImu(), 4.0, 0.6, left, right);
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
