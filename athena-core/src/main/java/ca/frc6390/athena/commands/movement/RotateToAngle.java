package ca.frc6390.athena.commands.movement;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.hardware.imu.Imu;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAngle extends Command{
    private static final double DEFAULT_TOLERANCE_RADIANS = Math.toRadians(2.5);

    private final RobotSpeeds speeds;
    private final Imu imu;
    private final Supplier<Pose2d> pose;
    private Supplier<Rotation2d> angle;
    private ProfiledPIDController rotationPID;
    private final DelayedOutput delayedOutput;

    public RotateToAngle(RobotCore<?> base, Supplier<Rotation2d> angle){
        this.speeds = base.drivetrain().robotSpeeds();
        this.imu = base.drivetrain().imu().device();
        this.angle = angle;
        this.pose = () -> base.localization().pose();

        this.rotationPID = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPID.setTolerance(DEFAULT_TOLERANCE_RADIANS);
        delayedOutput = new DelayedOutput(rotationPID::atSetpoint, 1);
    }

    public RotateToAngle(RobotCore<?> base, Rotation2d angle){
       this(base, () -> angle);
    }

    public RotateToAngle(RobotCore<?> base, DoubleSupplier angle){
        this(base, () -> Rotation2d.fromDegrees(angle.getAsDouble()));
    }

    public RotateToAngle(RobotCore<?> base, double degree){
        this(base, Rotation2d.fromDegrees(degree));
    }

    public RotateToAngle(RobotCore<?> base){
        this(base, () -> base.localization().pose().getRotation());
    }

    public RotateToAngle setPID(ProfiledPIDController rotationPID){
        if (rotationPID == null) {
            throw new IllegalArgumentException("rotationPID cannot be null");
        }
        this.rotationPID = rotationPID;
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPID.setTolerance(DEFAULT_TOLERANCE_RADIANS);
        return this;
    }

    public RotateToAngle setSupplierDegrees(Function<Pose2d, Double> func){
        if (func == null) {
            throw new IllegalArgumentException("func cannot be null");
        }
        this.angle = () -> Rotation2d.fromDegrees(func.apply(pose.get()));
        return this;
    }

    public RotateToAngle setSupplierRadians(Function<Pose2d, Double> func){
        if (func == null) {
            throw new IllegalArgumentException("func cannot be null");
        }
        this.angle = () -> Rotation2d.fromRadians(func.apply(pose.get()));
        return this;
    }

    @Override
    public void initialize() {
        rotationPID.reset(pose.get().getRotation().getRadians(), imu.getVelocityZ().getRadians());
    }

    @Override
    public void execute() {
        Rotation2d target = angle.get();
        if (target == null) {
            speeds.stopSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
            return;
        }
        double turnRate = rotationPID.calculate(pose.get().getRotation().getRadians(), target.getRadians());
        double maxAngularVelocity = speeds.getMaxAngularVelocity();
        if (Double.isFinite(maxAngularVelocity) && maxAngularVelocity > 0.0) {
            turnRate = MathUtil.clamp(turnRate, -maxAngularVelocity, maxAngularVelocity);
        }
        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0, 0, turnRate);
    }

    @Override
    public void end(boolean interrupted) {
        speeds.stopSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
    }

    @Override
    public boolean isFinished() {
        return delayedOutput.getAsBoolean();
    }
}
