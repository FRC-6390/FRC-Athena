package ca.frc6390.athena.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

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
    driveTrain.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.getMaxVelocity();
    double ySpeed =  yInput.getAsDouble() * driveTrain.getMaxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.getMaxAngularVelocity();

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, driveTrain.getRotation2d());

    driveTrain.drive(chassisSpeeds);    
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}