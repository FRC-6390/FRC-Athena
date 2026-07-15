package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.EncoderSlot;
import ca.frc6390.athena.mechanism.core.MechanismTemplate;
import ca.frc6390.athena.mechanism.core.MotorSlot;
import ca.frc6390.athena.mechanism.core.Slots;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import java.util.Objects;

/**
 * Base type for a swerve module declared as a specialized mechanism.
 */
@SuppressWarnings("this-escape")
public abstract class SwerveModule implements MechanismTemplate {
    private static final double TWO_PI = Math.PI * 2.0;
    private static final double HOLD_SPEED_EPSILON = 1.0e-3;
    private final SwerveModuleModel model;
    public final MotorSlot<SwerveModule> drive = Slots.motor(this, "drive", this::configureIfReady);
    public final MotorSlot<SwerveModule> steer = Slots.motor(this, "steer", this::configureIfReady);
    public final EncoderSlot<SwerveModule> angle = Slots.encoder(this, "angle", this::configureIfReady);
    public ControlBinding driveVelocity;
    public ControlBinding steerPosition;
    private double driveMaxSpeedMetersPerSecond = Double.NaN;
    private PidGains steerPid;
    private double heldAngleRotations = Double.NaN;

    protected SwerveModule(SwerveModuleModel model) {
        this.model = Objects.requireNonNull(model, "model");
    }

    public SwerveModuleModel model() {
        return model;
    }

    public SwerveModule driveMaxSpeedMetersPerSecond(double maxSpeedMetersPerSecond) {
        if (!Double.isFinite(maxSpeedMetersPerSecond) || maxSpeedMetersPerSecond <= 0.0) {
            throw new IllegalArgumentException("Drive max speed must be positive.");
        }
        driveMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        configureIfReady();
        return this;
    }

    public SwerveModule steerPid(double p, double i, double d) {
        steerPid = PidGains.of(p, i, d);
        configureIfReady();
        return this;
    }

    private void configureIfReady() {
        if (!drive.filled() || !steer.filled() || !angle.filled()) {
            return;
        }

        EncoderDevice driveDistance = drive.get().encoder()
                .gearRatio(GearRatio.reduction(model.driveReduction(), 1.0))
                .wheelDiameterMeters(model.wheelDiameterMeters())
                .units(EncoderUnit.METERS);

        driveVelocity = Controls.velocity(drive.get())
                .feedback(driveDistance);
        steerPosition = Controls.position(steer.get())
                .feedback(angle.get().absoluteSignal(), angle.get())
                .planner(MotionPlanners.boundedAngular(1.0));
        if (Double.isFinite(driveMaxSpeedMetersPerSecond)) {
            driveVelocity = driveVelocity.ff(0.0, 12.0 / driveMaxSpeedMetersPerSecond, 0.0);
        }
        if (steerPid != null) {
            steerPosition = steerPosition.pid(steerPid);
        }
    }

    public Action target(SwerveModuleTarget target) {
        Objects.requireNonNull(target, "target");
        if (driveVelocity == null || steerPosition == null) {
            throw new IllegalStateException("Swerve module controls have not been configured.");
        }
        if (driveVelocity.loops().isEmpty() || steerPosition.loops().isEmpty()) {
            throw new IllegalStateException(
                    "Swerve module requires driveMaxSpeedMetersPerSecond(...) and steerPid(...) before targeting.");
        }
        return Actions.computeHardware(
                java.util.List.of(drive.get(), steer.get(), angle.get()),
                context -> RuntimeHardwareAccess.call(context, () -> applyTarget(optimizedTarget(
                        target,
                        angle.get().absolutePosition()))));
    }

    SwerveModuleTarget optimizedTarget(SwerveModuleTarget target, double currentAngleRotations) {
        if (!Double.isFinite(currentAngleRotations)) {
            return new SwerveModuleTarget(0.0, Double.isFinite(heldAngleRotations) ? heldAngleRotations : 0.0);
        }
        if (Math.abs(target.speedMetersPerSecond()) < HOLD_SPEED_EPSILON) {
            if (!Double.isFinite(heldAngleRotations)) {
                heldAngleRotations = currentAngleRotations;
            }
            return new SwerveModuleTarget(0.0, heldAngleRotations);
        }

        double delta = wrapHalfRotation(target.angleRotations() - currentAngleRotations);
        double speed = target.speedMetersPerSecond();
        if (Math.abs(delta) > 0.25) {
            delta -= Math.copySign(0.5, delta);
            speed = -speed;
        }
        speed *= Math.cos(delta * TWO_PI);
        heldAngleRotations = currentAngleRotations + delta;
        return new SwerveModuleTarget(speed, heldAngleRotations);
    }

    private Action applyTarget(SwerveModuleTarget target) {
        return Actions.parallel(
                driveVelocity.velocity(target.speedMetersPerSecond()),
                steerPosition.position(target.angleRotations()));
    }

    private static double wrapHalfRotation(double rotations) {
        double wrapped = rotations - Math.floor(rotations + 0.5);
        return wrapped == -0.5 ? 0.5 : wrapped;
    }
}
