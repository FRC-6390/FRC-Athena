package ca.frc6390.athena.commands;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {

  //Creates a drivetrain subsystem
  private SwerveDrivetrain driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, yInput, thetaInput;

  public SwerveDriveCommand(SwerveDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    addRequirements(driveTrain);
  }
  
  @Override
  public void initialize() {
    driveTrain.setNeutralMode(MotorNeutralMode.Brake);
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.getMaxVelocity();
    double ySpeed =  yInput.getAsDouble() * driveTrain.getMaxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.getMaxVelocity();

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, driveTrain.getIMU().getVirtualAxis("driver"));

    driveTrain.getRobotSpeeds().setDriverSpeeds(chassisSpeeds);    
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.getRobotSpeeds().setDriverSpeeds(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}