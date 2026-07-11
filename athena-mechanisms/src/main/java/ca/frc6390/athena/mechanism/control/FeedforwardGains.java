package ca.frc6390.athena.mechanism.control;

import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopRole;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Output;

/**
 * Reusable feedforward gains whose calculated effort is expressed in volts.
 */
public record FeedforwardGains(
        double staticGain,
        double velocityGain,
        double accelerationGain,
        double gravityGain) implements ControlLoop {
    public FeedforwardGains {
        requireFinite(staticGain, "Feedforward static gain");
        requireFinite(velocityGain, "Feedforward velocity gain");
        requireFinite(accelerationGain, "Feedforward acceleration gain");
        requireFinite(gravityGain, "Feedforward gravity gain");
    }

    public FeedforwardGains(double staticGain, double velocityGain, double gravityGain) {
        this(staticGain, velocityGain, 0.0, gravityGain);
    }

    public static FeedforwardGains of(double staticGain, double velocityGain, double gravityGain) {
        return new FeedforwardGains(staticGain, velocityGain, gravityGain);
    }

    public static FeedforwardGains of(
            double staticGain,
            double velocityGain,
            double accelerationGain,
            double gravityGain) {
        return new FeedforwardGains(staticGain, velocityGain, accelerationGain, gravityGain);
    }

    public static FeedforwardGains simple(double staticGain, double velocityGain, double gravityGain) {
        return of(staticGain, velocityGain, gravityGain);
    }

    public FeedforwardGains acceleration(double accelerationGain) {
        return new FeedforwardGains(staticGain, velocityGain, accelerationGain, gravityGain);
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(this);
    }

    @Override
    public ControlLoopRole role() {
        return ControlLoopRole.DEVICE_CONFIGURABLE;
    }

    private static void requireFinite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
    }

    private static final class Runtime implements ControlLoopRuntime {
        private final FeedforwardGains gains;

        private Runtime(FeedforwardGains gains) {
            this.gains = gains;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double velocity = context.reference().velocity();
            double acceleration = context.reference().acceleration();
            if (!Double.isFinite(velocity) || !Double.isFinite(acceleration)) {
                return ControlOutput.neutral();
            }
            double sign = velocity == 0.0 && context.request() instanceof Output.Position
                    ? Math.signum(context.target() - context.position())
                    : Math.signum(velocity);
            double volts = gains.staticGain * sign
                    + gains.velocityGain * velocity
                    + gains.accelerationGain * acceleration
                    + gains.gravityGain;
            return Double.isFinite(volts) ? ControlOutput.voltage(volts) : ControlOutput.neutral();
        }
    }
}
