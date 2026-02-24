package ca.frc6390.athena.commands.control;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDriveCommand extends Command {

  //Creates a drivetrain subsystem
  private DifferentialDrivetrain driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, thetaInput;

  public TankDriveCommand(DifferentialDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier thetaInput) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.thetaInput = thetaInput;
    addRequirements(driveTrain);
    driveTrain.hardware().neutralMode(MotorNeutralMode.Brake);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.speeds().maxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.speeds().maxAngularVelocity();


    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, thetaSpeed);

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
