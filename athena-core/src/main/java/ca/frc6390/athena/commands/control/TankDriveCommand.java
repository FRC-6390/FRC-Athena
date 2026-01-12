package ca.frc6390.athena.commands.control;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
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
    driveTrain.setNeutralMode(MotorNeutralMode.Brake);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();


    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, thetaSpeed);

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
