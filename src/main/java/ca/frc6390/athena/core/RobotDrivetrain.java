package ca.frc6390.athena.core;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import edu.wpi.first.wpilibj2.command.Command;

public interface RobotDrivetrain {

    public record RobotDrivetrainConfig() {
    }

    IMU getIMU();
    MotorNeutralMode getNeutralMode();
    void setNeutralMode(MotorNeutralMode mode);
    RobotSpeeds getRobotSpeeds();
    void update();
    Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);
    void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);
}
