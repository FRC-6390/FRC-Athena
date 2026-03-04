package ca.frc6390.athena.commands.control;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.Objects;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
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
    this.fieldRelativeSupplier = Objects.requireNonNull(fieldRelativeSupplier, "fieldRelativeSupplier");
    addRequirements(driveTrain);
    driveTrain.hardware().neutralMode(MotorNeutralMode.Brake);
  }
  
  @Override
  public void initialize() 
  {


  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.speeds().maxVelocity();
    double ySpeed = yInput.getAsDouble() * driveTrain.speeds().maxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.speeds().maxAngularVelocity();

    // Keep command-space shaping linear. Skew correction is handled once in drivetrain update()
    // via ChassisSpeeds.discretize() immediately before kinematics.
    ChassisSpeeds chassisSpeeds = computeChassisSpeeds(
        xSpeed,
        ySpeed,
        thetaSpeed,
        fieldRelativeSupplier.getAsBoolean(),
        driveTrain.imu().device().getVirtualAxis("driver"));

    driveTrain.speeds().set(RobotSpeeds.DRIVE_SOURCE, chassisSpeeds);
  }

  static ChassisSpeeds computeChassisSpeeds(
      double xSpeed,
      double ySpeed,
      double thetaSpeed,
      boolean fieldRelative,
      Rotation2d driverHeading) {
    if (fieldRelative) {
      Rotation2d resolvedHeading = driverHeading != null ? driverHeading : Rotation2d.kZero;
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          thetaSpeed,
          resolvedHeading);
    }
    return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.clearDriveCommandInputs();
    driveTrain.speeds().stop(RobotSpeeds.DRIVE_SOURCE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
