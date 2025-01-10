package ca.frc6390.athena.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {


  public record SwerveDriveCommandConfig(double thetaDeadzone, double maxVelocity, double maxAngularVelocity, double maxAcceleration, double maxAngularAcceleration) {}

  //Creates a drivetrain subsystem
  private SwerveDrivetrain driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, yInput, thetaInput;
  //These are limiters. The make sure the rate of change is never too abrupt and smooth out inputs from the joystick.
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  private SwerveDriveCommandConfig config;

  public SwerveDriveCommand(SwerveDrivetrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput, SwerveDriveCommandConfig config) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.config = config;
    xLimiter = new SlewRateLimiter(config.maxAcceleration());
    yLimiter = new SlewRateLimiter(config.maxAcceleration());
    thetaLimiter = new SlewRateLimiter(config.maxAngularAcceleration());
    addRequirements(driveTrain);

    this.thetaInput = () -> {return Math.abs(thetaInput.getAsDouble()) > config.thetaDeadzone() ? thetaInput.getAsDouble() : 0;};
  }

  @Override
  public void initialize() {
    driveTrain.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void execute() {
    
    double xSpeed = xLimiter.calculate(xInput.getAsDouble()) * config.maxVelocity();
    double ySpeed = yLimiter.calculate(yInput.getAsDouble()) * config.maxVelocity();
    double thetaSpeed = thetaLimiter.calculate(thetaInput.getAsDouble()) * config.maxAngularVelocity();

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