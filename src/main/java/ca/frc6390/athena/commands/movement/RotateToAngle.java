package ca.frc6390.athena.commands.movement;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.devices.IMU;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAngle extends Command{

    private final RobotSpeeds speeds;
    private final IMU imu;
    private final Supplier<Pose2d> pose;
    private Supplier<Rotation2d> angle;
    private ProfiledPIDController rotationPID;
    private final DelayedOutput delayedOutput;

    public RotateToAngle(RobotBase<?> base, Supplier<Rotation2d> angle, boolean relative){
        this.speeds = base.getDrivetrain().getRobotSpeeds();
        this.imu = base.getDrivetrain().getIMU();
        this.angle = angle;
        this.pose = relative ? () -> base.getLocalization().getRelativePose() : () -> base.getLocalization().getFieldPose();

        this.rotationPID = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPID.setTolerance(2.5);
        delayedOutput = new DelayedOutput(rotationPID::atSetpoint, 1);
    }

    public RotateToAngle(RobotBase<?> base, Rotation2d angle, boolean relative){
       this(base, () -> angle, relative);
    }

    public RotateToAngle(RobotBase<?> base, DoubleSupplier angle, boolean relative){
        this(base, () -> Rotation2d.fromDegrees(angle.getAsDouble()), relative);
    }

    public RotateToAngle(RobotBase<?> base, double degree, boolean relative){
        this(base, Rotation2d.fromDegrees(degree), relative);
    }

    public RotateToAngle(RobotBase<?> base, boolean relative){
        this(base, () -> null, relative);
    }

    public RotateToAngle setPID(ProfiledPIDController rotationPID){
        this.rotationPID = rotationPID;
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPID.setTolerance(0.05);
        return this;
    }

    public RotateToAngle setSupplier(Function<Pose2d, Double> func){
        this.angle = () -> Rotation2d.fromDegrees(func.apply(pose.get()));
        return this;
    }

    @Override
    public void initialize() {
        rotationPID.reset(pose.get().getRotation().getDegrees(), imu.getVelocityZ().getDegrees());
    }

    @Override
    public void execute() {
        double turnRate = rotationPID.calculate(pose.get().getRotation().getDegrees(), angle.get().getDegrees());
        speeds.setSpeeds("feedback", 0, 0, turnRate);
    }

    @Override
    public void end(boolean interrupted) {
        speeds.stopSpeeds("feedbackl");
    }

    @Override
    public boolean isFinished() {
        return delayedOutput.getAsBoolean();
    }
}
