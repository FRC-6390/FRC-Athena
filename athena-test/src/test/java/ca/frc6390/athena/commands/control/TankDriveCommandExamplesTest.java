package ca.frc6390.athena.commands.control;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.commands.examples.TankDriveCommandExamples;
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

final class TankDriveCommandExamplesTest {

    @Test
    void arcadeDriveScalesInputsAndStopsOnEnd() {
        TestDrivetrainContext context = createDrivetrain(4.0);
        TankDriveCommand command = TankDriveCommandExamples.arcadeDrive(
                context.drivetrain,
                () -> 0.5,
                () -> -0.25);

        assertEquals(MotorNeutralMode.Brake, context.leftA.neutralMode);
        assertEquals(MotorNeutralMode.Brake, context.rightA.neutralMode);

        command.execute();
        ChassisSpeeds input = context.drivetrain.speeds().getInput("drive");
        assertEquals(2.0, input.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, input.vyMetersPerSecond, 1e-9);
        assertEquals(-1.0, input.omegaRadiansPerSecond, 1e-9);

        command.end(false);
        ChassisSpeeds stopped = context.drivetrain.speeds().getInput("drive");
        assertEquals(0.0, stopped.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, stopped.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, stopped.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void arcadeDriveWithDeadbandSuppressesSmallInputs() {
        TestDrivetrainContext context = createDrivetrain(5.0);
        TankDriveCommand command = TankDriveCommandExamples.arcadeDriveWithDeadband(
                context.drivetrain,
                () -> 0.04,
                () -> -0.08,
                0.1);

        command.execute();
        ChassisSpeeds input = context.drivetrain.speeds().getInput("drive");
        assertEquals(0.0, input.vxMetersPerSecond, 1e-9);
        assertEquals(0.0, input.vyMetersPerSecond, 1e-9);
        assertEquals(0.0, input.omegaRadiansPerSecond, 1e-9);
    }

    private static TestDrivetrainContext createDrivetrain(double maxVelocityMetersPerSecond) {
        FakeMotor leftA = new FakeMotor(1);
        FakeMotor leftB = new FakeMotor(2);
        FakeMotor rightA = new FakeMotor(3);
        FakeMotor rightB = new FakeMotor(4);

        MotorControllerGroup left = new MotorControllerGroup(leftA, leftB);
        MotorControllerGroup right = new MotorControllerGroup(rightA, rightB);
        DifferentialDrivetrain drivetrain =
                new DifferentialDrivetrain(new FakeImu(), maxVelocityMetersPerSecond, 0.6, left, right);

        return new TestDrivetrainContext(drivetrain, leftA, rightA);
    }

    private record TestDrivetrainContext(DifferentialDrivetrain drivetrain, FakeMotor leftA, FakeMotor rightA) {}

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
        private MotorNeutralMode neutralMode = MotorNeutralMode.Coast;

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
            neutralMode = mode;
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
