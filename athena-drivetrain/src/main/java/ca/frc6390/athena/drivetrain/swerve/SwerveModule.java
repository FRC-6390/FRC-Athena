package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.EncoderSlot;
import ca.frc6390.athena.mechanism.core.MechanismTemplate;
import ca.frc6390.athena.mechanism.core.MotorSlot;
import ca.frc6390.athena.mechanism.core.Slots;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.control.TorqueVelocityGains;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import java.util.Objects;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

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
    public ControlBinding<?> driveVelocity;
    public ControlBinding<?> steerPosition;
    private double driveMaxSpeedMetersPerSecond = Double.NaN;
    private double driveMaxAccelerationMetersPerSecondSquared = Double.NaN;
    private TorqueVelocityGains driveTorque;
    private double steerDeadbandRotations;
    private PidGains steerPid;
    private double heldAngleRotations = Double.NaN;

    protected SwerveModule(SwerveModuleModel model) {
        this.model = Objects.requireNonNull(model, "model");
    }

    public SwerveModuleModel model() {
        return model;
    }

    /** Sets the wheel speed that maps to full drive voltage. Used as the drive velocity feedforward scale. */
    public SwerveModule driveMaxSpeed(LinearVelocity maxSpeed) {
        double maxSpeedMetersPerSecond = Objects.requireNonNull(maxSpeed, "maxSpeed").in(MetersPerSecond);
        if (!Double.isFinite(maxSpeedMetersPerSecond) || maxSpeedMetersPerSecond <= 0.0) {
            throw new IllegalArgumentException("Drive max speed must be positive.");
        }
        driveMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        configureIfReady();
        return this;
    }

    /**
     * Drives the wheel with software torque control instead of the {@code 12 V / driveMaxSpeed} voltage
     * feedforward: the wheel-speed error and the reference acceleration become a motor current, turned into a
     * voltage with the motor model ({@code V = I R + omega / Kv}). This needs no motor license. Force, and so
     * acceleration, then follows the request at any battery voltage, and the drive stops fighting a voltage step
     * whenever the target changes. Gains are in amps per m/s (kP) and amps per m/s^2 ({@code inertia}): for one
     * motor per module, {@code inertia = robot mass / 4 * wheel radius / (gear ratio * kT)}.
     *
     * @param gains torque-velocity gains for this module's drive motor
     * @return this module
     */
    public SwerveModule driveTorque(TorqueVelocityGains gains) {
        driveTorque = Objects.requireNonNull(gains, "gains");
        configureIfReady();
        return this;
    }

    /**
     * Limits how fast this wheel's speed reference may change, a per-wheel traction limit that also applies to
     * paths and aiming. With {@link #driveTorque} the limited reference also supplies the acceleration for its
     * inertia term. Set it near what the wheels can grip (about 1 g on carpet).
     *
     * @param maxAcceleration wheel acceleration limit, positive
     * @return this module
     */
    public SwerveModule driveMaxAcceleration(LinearAcceleration maxAcceleration) {
        double value = Objects.requireNonNull(maxAcceleration, "maxAcceleration").in(MetersPerSecondPerSecond);
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException("Drive max acceleration must be positive.");
        }
        driveMaxAccelerationMetersPerSecondSquared = value;
        configureIfReady();
        return this;
    }

    /**
     * Leaves the steering alone while the wheel is within {@code deadband} of its target angle: the module is
     * commanded to stay where it is, so the steer motor does not pulse current into corrections of a fraction
     * of a degree (a source of buzzing and wasted power). The wheel speed is still scaled by the actual angle
     * error. Use zero to turn it off (the default); about half a degree suits most modules.
     *
     * @param deadband angle band, non-negative
     * @return this module
     */
    public SwerveModule steerDeadband(edu.wpi.first.units.measure.Angle deadband) {
        double rotations = Objects.requireNonNull(deadband, "deadband").in(Units.Rotations);
        if (!Double.isFinite(rotations) || rotations < 0.0) {
            throw new IllegalArgumentException("Steer deadband must be finite and non-negative.");
        }
        steerDeadbandRotations = rotations;
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
                .reduction(model.driveReduction())
                .measures(Units.Meters)
                .travelPerRevolution(model.wheelCircumference());

        driveVelocity = Controls.velocity(drive.get(), Units.Meters)
                .feedback(driveDistance);
        steerPosition = Controls.position(steer.get(), Units.Rotations)
                .feedback(angle.get().absoluteSignal(), angle.get().velocitySignal())
                .planner(MotionPlanners.boundedAngular(Units.Rotations.of(1.0), Units.Rotations));
        if (driveTorque != null) {
            driveVelocity = driveVelocity.torqueVelocity(driveTorque);
        } else if (Double.isFinite(driveMaxSpeedMetersPerSecond)) {
            driveVelocity = driveVelocity.ff(0.0, 12.0 / driveMaxSpeedMetersPerSecond, 0.0);
        }
        if (Double.isFinite(driveMaxAccelerationMetersPerSecondSquared)) {
            // The speed bound only needs to be above anything the drive asks for; the acceleration is the limit.
            double speedBound = Double.isFinite(driveMaxSpeedMetersPerSecond)
                    ? driveMaxSpeedMetersPerSecond * 2.0
                    : 100.0;
            driveVelocity = driveVelocity.constraints(Constraints.motion(
                    MetersPerSecond.of(speedBound),
                    MetersPerSecondPerSecond.of(driveMaxAccelerationMetersPerSecondSquared)));
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
                    "Swerve module requires driveMaxSpeed(...) and steerPid(...) before targeting.");
        }
        return Actions.computeHardware(
                java.util.List.of(drive.get(), steer.get(), angle.get()),
                context -> RuntimeHardwareAccess.call(context, () -> applyTarget(optimizedTarget(
                        target,
                        angle.get().raw().absolutePosition()))));
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
        // Within the deadband, hold the angle the module already has instead of correcting a fraction of a degree.
        heldAngleRotations = Math.abs(delta) <= steerDeadbandRotations
                ? currentAngleRotations
                : currentAngleRotations + delta;
        return new SwerveModuleTarget(speed, heldAngleRotations);
    }

    /** Drive voltage with the steer held at zero. Used only by drive characterization. */
    Action characterizeDrive(double volts) {
        if (driveVelocity == null || steerPosition == null) {
            throw new IllegalStateException("Swerve module controls have not been configured.");
        }
        return Actions.parallel(driveVelocity.voltage(Units.Volts.of(volts)),
                steerPosition.position(Units.Rotations.of(0.0)));
    }

    private Action applyTarget(SwerveModuleTarget target) {
        return Actions.parallel(
                driveVelocity.velocity(Units.MetersPerSecond.of(target.speedMetersPerSecond())),
                steerPosition.position(Units.Rotations.of(target.angleRotations())));
    }

    private static double wrapHalfRotation(double rotations) {
        double wrapped = rotations - Math.floor(rotations + 0.5);
        return wrapped == -0.5 ? 0.5 : wrapped;
    }
}
