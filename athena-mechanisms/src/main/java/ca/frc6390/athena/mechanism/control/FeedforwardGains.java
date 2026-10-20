package ca.frc6390.athena.mechanism.control;

import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRole;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlLoopTrace;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Output;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.Map;
import java.util.Objects;

/**
 * Feedforward gains automatically exposed for live editing with their owning mechanism.
 *
 * <p>Two gravity models share this class. The plain form ({@link #of(double, double, double)}) adds
 * {@code kG} as a constant voltage, which is right for an elevator or a turret. The arm form
 * ({@link #arm(double, double, double)}) adds {@code kG * cos(angle + offset)}, where the angle is the
 * mechanism position in radians measured from the encoder reference. The angle unit comes from the
 * control binding, so it is bound when the arm form is declared on a binding.</p>
 */
public final class FeedforwardGains implements ControlLoop, TelemetrySource {
    private volatile double staticGain;
    private volatile double velocityGain;
    private volatile double accelerationGain;
    private volatile double gravityGain;
    private volatile boolean disabled;
    /** True for the arm form, whose gravity term depends on the position angle. */
    private final boolean arm;
    /** Phase added to the angle (radians) before the cosine; zero for the plain form. */
    private final double gravityOffsetRadians;
    /** Angle unit the position is measured in; null until a control binding binds the arm form. */
    private final AngleUnit gravityAngle;
    private final Map<String, TelemetryValue> telemetry = Map.of(
            "static", TelemetryValue.writableNumber(this::staticGain, this::setStatic),
            "velocity", TelemetryValue.writableNumber(this::velocityGain, this::setVelocity),
            "acceleration", TelemetryValue.writableNumber(this::accelerationGain, this::setAcceleration),
            "gravity", TelemetryValue.writableNumber(this::gravityGain, this::setGravity),
            "disabled", TelemetryValue.writableBoolean(this::isDisabled, value -> disabled = value));

    public FeedforwardGains(double s, double v, double a, double g) {
        this(s, v, a, g, false, 0.0, null);
    }

    private FeedforwardGains(double s, double v, double a, double g, boolean arm, double offset, AngleUnit angle) {
        setStatic(s); setVelocity(v); setAcceleration(a); setGravity(g);
        this.arm = arm;
        this.gravityOffsetRadians = finite(offset, "Arm gravity offset");
        this.gravityAngle = angle;
    }
    public FeedforwardGains(double s, double v, double g) { this(s, v, 0.0, g); }
    public static FeedforwardGains of(double s, double v, double g) { return new FeedforwardGains(s, v, g); }
    public static FeedforwardGains of(double s, double v, double a, double g) { return new FeedforwardGains(s, v, a, g); }
    public static FeedforwardGains simple(double s, double v, double g) { return of(s, v, g); }

    /**
     * Arm feedforward: {@code kS * sign + kV * v + kG * cos(angle)} with the angle in radians measured
     * from the encoder reference. Use this for a single-jointed arm, whose gravity torque follows
     * {@code cos} of the angle from horizontal. The angle unit is bound by the control binding that
     * receives the gains; declaring it on a velocity control or with an unknown angle unit is rejected.
     *
     * @param s static gain in volts
     * @param v velocity gain in volts per unit per second
     * @param g gravity amplitude in volts, at the angle where gravity is strongest
     * @return arm feedforward gains with no offset
     */
    public static FeedforwardGains arm(double s, double v, double g) {
        return new FeedforwardGains(s, v, 0.0, g, true, 0.0, null);
    }

    /**
     * Arm feedforward with a phase offset: {@code kG * cos(angle + offset)}. The offset is added to the
     * angle, so the sum is the angle from horizontal when the encoder reads {@code -offset} there. The convention matches {@code MechanismModel.gravityVolts} for rotary models.
     *
     * @param s static gain in volts
     * @param v velocity gain in volts per unit per second
     * @param g gravity amplitude in volts
     * @param gravityOffset phase added to the angle; must be finite (any angle unit, stored in radians)
     * @return arm feedforward gains
     */
    public static FeedforwardGains arm(double s, double v, double g, Angle gravityOffset) {
        return new FeedforwardGains(s, v, 0.0, g, true, Objects.requireNonNull(gravityOffset, "gravityOffset")
                .in(Units.Radians), null);
    }

    public double staticGain() { return staticGain; }
    public double velocityGain() { return velocityGain; }
    public double accelerationGain() { return accelerationGain; }
    public double gravityGain() { return gravityGain; }
    public boolean isDisabled() { return disabled; }
    /** True for the arm form; false means gravity is a constant voltage. */
    public boolean isArm() { return arm; }
    /** Phase added to the angle before the gravity cosine (arm form only; zero for the plain form). */
    public Angle gravityOffset() { return Units.Radians.of(gravityOffsetRadians); }
    /** Angle unit the arm position is measured in, or null while the gains are not yet bound to a binding. */
    public AngleUnit gravityAngle() { return gravityAngle; }

    /**
     * Returns these arm gains bound to the angle unit that the position is expressed in. Control bindings
     * call this when the arm form is added; a user normally does not.
     *
     * @param angle angle unit of the control position
     * @return arm gains with the angle unit set
     * @throws IllegalStateException when these gains are not the arm form
     */
    public FeedforwardGains withGravityAngle(AngleUnit angle) {
        if (!arm) {
            throw new IllegalStateException("Only arm feedforward gains take a gravity angle.");
        }
        return new FeedforwardGains(staticGain, velocityGain, accelerationGain, gravityGain, true,
                gravityOffsetRadians, Objects.requireNonNull(angle, "angle"));
    }

    public FeedforwardGains acceleration(double value) {
        return new FeedforwardGains(staticGain, velocityGain, value, gravityGain, arm, gravityOffsetRadians,
                gravityAngle);
    }
    @Override public ControlLoopRuntime bind(ControlLoopBinding binding) { return new Runtime(); }
    @Override public ControlLoopRole role() {
        // The arm form adds a voltage to the loop, so it can ride on a device-owned loop as arbitrary
        // feedforward. A constant gravity term stays in the device's own gains.
        return arm ? ControlLoopRole.ARBITRARY_FEEDFORWARD : ControlLoopRole.DEVICE_CONFIGURABLE;
    }

    @Override public Map<String, TelemetryValue> telemetry() {
        return telemetry;
    }
    private void setStatic(double value) { staticGain = finite(value, "Feedforward static gain"); }
    private void setVelocity(double value) { velocityGain = finite(value, "Feedforward velocity gain"); }
    private void setAcceleration(double value) { accelerationGain = finite(value, "Feedforward acceleration gain"); }
    private void setGravity(double value) { gravityGain = finite(value, "Feedforward gravity gain"); }
    private static double finite(double value, String description) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(description + " must be finite.");
        return value;
    }

    @Override public boolean equals(Object other) {
        return other instanceof FeedforwardGains gains
                && Double.compare(staticGain, gains.staticGain) == 0
                && Double.compare(velocityGain, gains.velocityGain) == 0
                && Double.compare(accelerationGain, gains.accelerationGain) == 0
                && Double.compare(gravityGain, gains.gravityGain) == 0
                && arm == gains.arm
                && Double.compare(gravityOffsetRadians, gains.gravityOffsetRadians) == 0
                && Objects.equals(gravityAngle, gains.gravityAngle);
    }
    @Override public int hashCode() {
        return Objects.hash(staticGain, velocityGain, accelerationGain, gravityGain, arm,
                gravityOffsetRadians, gravityAngle);
    }
    @Override public String toString() {
        return "FeedforwardGains[staticGain=" + staticGain + ", velocityGain=" + velocityGain
                + ", accelerationGain=" + accelerationGain + ", gravityGain=" + gravityGain
                + (arm ? ", arm=true, gravityOffsetRadians=" + gravityOffsetRadians
                        + ", gravityAngle=" + (gravityAngle == null ? "unbound" : gravityAngle.name()) : "")
                + "]";
    }

    private final class Runtime implements ControlLoopRuntime {
        private ControlLoopTrace trace = ControlLoopTrace.ZERO;
        private double seenStatic = staticGain;
        private double seenVelocity = velocityGain;
        private double seenAcceleration = accelerationGain;
        private double seenGravity = gravityGain;

        @Override public boolean consumeGainChange() {
            boolean changed = staticGain != seenStatic || velocityGain != seenVelocity
                    || accelerationGain != seenAcceleration || gravityGain != seenGravity;
            seenStatic = staticGain;
            seenVelocity = velocityGain;
            seenAcceleration = accelerationGain;
            seenGravity = gravityGain;
            return changed;
        }

        @Override public ControlOutput calculate(ControlLoopContext context) {
            if (disabled) return ControlOutput.voltage(0.0);
            double velocity = context.reference().velocity();
            double acceleration = context.reference().acceleration();
            if (!Double.isFinite(velocity) || !Double.isFinite(acceleration)) return ControlOutput.neutral();
            double gravity = gravityGain;
            if (arm) {
                if (gravityAngle == null) {
                    throw new IllegalStateException("Arm feedforward is not bound to a control angle unit.");
                }
                double position = context.position();
                if (!Double.isFinite(position)) return ControlOutput.neutral();
                gravity = gravityGain * Math.cos(gravityAngle.toBaseUnits(position) + gravityOffsetRadians);
            }
            double sign = velocity == 0.0 && context.request() instanceof Output.Position
                    ? Math.signum(context.target() - context.position()) : Math.signum(velocity);
            trace = new ControlLoopTrace(0, 0, 0, staticGain * sign, velocityGain * velocity,
                    accelerationGain * acceleration, gravity);
            return ControlOutput.voltage(trace.staticFeedforwardVolts() + trace.velocityFeedforwardVolts()
                    + trace.accelerationFeedforwardVolts() + trace.gravityFeedforwardVolts());
        }
        @Override public void reset(ControlLoopContext context) { trace = ControlLoopTrace.ZERO; }
        @Override public ControlLoopTrace trace() { return trace; }
    }
}
