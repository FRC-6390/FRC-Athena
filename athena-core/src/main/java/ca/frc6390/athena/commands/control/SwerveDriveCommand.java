package ca.frc6390.athena.commands.control;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.Objects;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
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
  public void initialize() {
    driveTrain.bindDriveCommandInputs(xInput, yInput, thetaInput, fieldRelativeSupplier);
    driveTrain.applyBoundDriveCommandInputs();
  }

  @Override
  public void execute() {
    driveTrain.applyBoundDriveCommandInputs();
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
