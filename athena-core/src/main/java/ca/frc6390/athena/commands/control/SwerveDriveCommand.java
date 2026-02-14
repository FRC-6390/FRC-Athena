package ca.frc6390.athena.commands.control;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {

  //Creates a drivetrain subsystem
  private final SwerveDrivetrain driveTrain;
  private final boolean fieldRelative;
  //Double suppliers are outputted by the joystick
  private final DoubleSupplier xInput, yInput, thetaInput;

  public SwerveDriveCommand(SwerveDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput, boolean fieldRelative) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    this.fieldRelative = fieldRelative;
    addRequirements(driveTrain);
    driveTrain.hardware().neutralMode(MotorNeutralMode.Brake);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.speeds().maxVelocity();
    double ySpeed = yInput.getAsDouble() * driveTrain.speeds().maxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.speeds().maxAngularVelocity();

    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, driveTrain.imu().device().getVirtualAxis("driver"))
        : new ChassisSpeeds(ySpeed, xSpeed, thetaSpeed);

    driveTrain.speeds().set("drive", chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.speeds().stop("drive");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
