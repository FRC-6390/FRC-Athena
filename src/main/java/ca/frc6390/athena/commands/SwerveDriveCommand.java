package ca.frc6390.athena.commands;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {


  public record SwerverDriveCommandConfig(double thetaDeadzone, double maxAcceleration, double maxAngularAcceleration) {}

  //Creates a drivetrain subsystem
  private SwerveDrivetrain driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, yInput, thetaInput;
  //These are limiters. The make sure the rate of change is never too abrupt and smooth out inputs from the joystick.
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  private double maxAcceleration, maxAngularAcceleration;

  public SwerveDriveCommand(SwerveDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput, SwerverDriveCommandConfig config) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.maxAcceleration = config.maxAcceleration();
    this.maxAngularAcceleration = config.maxAngularAcceleration();
    xLimiter = new SlewRateLimiter(maxAcceleration);
    yLimiter = new SlewRateLimiter(maxAcceleration);
    thetaLimiter = new SlewRateLimiter(maxAngularAcceleration);
    addRequirements(driveTrain);

    this.thetaInput = () -> {return Math.abs(thetaInput.getAsDouble()) > config.thetaDeadzone() ? thetaInput.getAsDouble() : 0;};
  }

  @Override
  public void initialize() {
    driveTrain.lockWheels();
  }

  @Override
  public void execute() {
    
    double xSpeed = xLimiter.calculate(xInput.getAsDouble()) * maxAcceleration;
    double ySpeed = yLimiter.calculate(yInput.getAsDouble()) * maxAcceleration;
    double thetaSpeed = thetaLimiter.calculate(thetaInput.getAsDouble()) * maxAngularAcceleration;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, thetaSpeed, driveTrain.getRotation2d());

    //Feed that into the drive train subsystem
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