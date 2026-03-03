package ca.frc6390.athena.commands.control;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {

  //Creates a drivetrain subsystem
  private final SwerveDrivetrain driveTrain;
  private final BooleanSupplier fieldRelativeSupplier;
  //Double suppliers are outputted by the joystick
  private final DoubleSupplier xInput, yInput, thetaInput;

  public SwerveDriveCommand(SwerveDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput, boolean fieldRelative) {
    this(driveTrain, xInput, yInput, thetaInput, () -> fieldRelative);
  }

  public SwerveDriveCommand(
      SwerveDrivetrain driveTrain,
      DoubleSupplier xInput,
      DoubleSupplier yInput,
      DoubleSupplier thetaInput,
      BooleanSupplier fieldRelativeSupplier) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
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

    ChassisSpeeds chassisSpeeds = fieldRelativeSupplier.getAsBoolean()
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, driveTrain.imu().device().getVirtualAxis("driver"))
        : new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

    driveTrain.speeds().set(RobotSpeeds.DRIVE_SOURCE, chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.speeds().stop(RobotSpeeds.DRIVE_SOURCE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
