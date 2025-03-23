package ca.frc6390.athena.commands.control;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
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
  }
  
  @Override
  public void initialize() {
    driveTrain.setNeutralMode(MotorNeutralMode.Brake);
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();
    double ySpeed =  yInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();

    ChassisSpeeds chassisSpeeds = !fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, driveTrain.getIMU().getVirtualAxis("driver")) : ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, driveTrain.getIMU().getVirtualAxis("driver"));

    driveTrain.getRobotSpeeds().setSpeeds("drive", chassisSpeeds);    
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.getRobotSpeeds().stopSpeeds("drive");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}