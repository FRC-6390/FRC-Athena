package ca.frc6390.athena.commands.control;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.tank.TankDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDriveCommand extends Command {

  //Creates a drivetrain subsystem
  private TankDrivetrain driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, thetaInput;

  public TankDriveCommand(TankDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier thetaInput) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.thetaInput = thetaInput;
    addRequirements(driveTrain);
  }
  
  @Override
  public void initialize() {
    driveTrain.setNeutralMode(MotorNeutralMode.Brake);
  }

  @Override
  public void execute() {

    double xSpeed = xInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();
    double thetaSpeed = thetaInput.getAsDouble() * driveTrain.getRobotSpeeds().getMaxVelocity();

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, xSpeed, thetaSpeed, driveTrain.getIMU().getVirtualAxis("driver"));

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