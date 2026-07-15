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
import java.util.Map;
import java.util.Objects;

/** Feedforward gains automatically exposed for live editing with their owning mechanism. */
public final class FeedforwardGains implements ControlLoop, TelemetrySource {
    private volatile double staticGain;
    private volatile double velocityGain;
    private volatile double accelerationGain;
    private volatile double gravityGain;
    private volatile boolean disabled;
    private final Map<String, TelemetryValue> telemetry = Map.of(
            "static", TelemetryValue.writableNumber(this::staticGain, this::setStatic),
            "velocity", TelemetryValue.writableNumber(this::velocityGain, this::setVelocity),
            "acceleration", TelemetryValue.writableNumber(this::accelerationGain, this::setAcceleration),
            "gravity", TelemetryValue.writableNumber(this::gravityGain, this::setGravity),
            "disabled", TelemetryValue.writableBoolean(this::isDisabled, value -> disabled = value));

    public FeedforwardGains(double s, double v, double a, double g) {
        setStatic(s); setVelocity(v); setAcceleration(a); setGravity(g);
    }
    public FeedforwardGains(double s, double v, double g) { this(s, v, 0.0, g); }
    public static FeedforwardGains of(double s, double v, double g) { return new FeedforwardGains(s, v, g); }
    public static FeedforwardGains of(double s, double v, double a, double g) { return new FeedforwardGains(s, v, a, g); }
    public static FeedforwardGains simple(double s, double v, double g) { return of(s, v, g); }
    public double staticGain() { return staticGain; }
    public double velocityGain() { return velocityGain; }
    public double accelerationGain() { return accelerationGain; }
    public double gravityGain() { return gravityGain; }
    public boolean isDisabled() { return disabled; }
    public FeedforwardGains acceleration(double value) {
        return new FeedforwardGains(staticGain, velocityGain, value, gravityGain);
    }
    @Override public ControlLoopRuntime bind(ControlLoopBinding binding) { return new Runtime(); }
    @Override public ControlLoopRole role() { return ControlLoopRole.DEVICE_CONFIGURABLE; }

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
                && Double.compare(gravityGain, gains.gravityGain) == 0;
    }
    @Override public int hashCode() {
        return Objects.hash(staticGain, velocityGain, accelerationGain, gravityGain);
    }
    @Override public String toString() {
        return "FeedforwardGains[staticGain=" + staticGain + ", velocityGain=" + velocityGain
                + ", accelerationGain=" + accelerationGain + ", gravityGain=" + gravityGain + "]";
    }

    private final class Runtime implements ControlLoopRuntime {
        private ControlLoopTrace trace = ControlLoopTrace.ZERO;
        @Override public ControlOutput calculate(ControlLoopContext context) {
            if (disabled) return ControlOutput.voltage(0.0);
            double velocity = context.reference().velocity();
            double acceleration = context.reference().acceleration();
            if (!Double.isFinite(velocity) || !Double.isFinite(acceleration)) return ControlOutput.neutral();
            double sign = velocity == 0.0 && context.request() instanceof Output.Position
                    ? Math.signum(context.target() - context.position()) : Math.signum(velocity);
            trace = new ControlLoopTrace(0, 0, 0, staticGain * sign, velocityGain * velocity,
                    accelerationGain * acceleration, gravityGain);
            return ControlOutput.voltage(trace.staticFeedforwardVolts() + trace.velocityFeedforwardVolts()
                    + trace.accelerationFeedforwardVolts() + trace.gravityFeedforwardVolts());
        }
        @Override public void reset(ControlLoopContext context) { trace = ControlLoopTrace.ZERO; }
        @Override public ControlLoopTrace trace() { return trace; }
    }
}
