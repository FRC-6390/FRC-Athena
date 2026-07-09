package ca.frc6390.athena.mechanism.ref;

import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;

/**
 * Reusable feedforward gains for control loops.
 */
public record FeedforwardGains(double staticGain, double velocityGain, double gravityGain) implements ControlLoop {
    public static FeedforwardGains of(double staticGain, double velocityGain, double gravityGain) {
        return new FeedforwardGains(staticGain, velocityGain, gravityGain);
    }

    public static FeedforwardGains simple(double staticGain, double velocityGain, double gravityGain) {
        return of(staticGain, velocityGain, gravityGain);
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(this);
    }

    private static final class Runtime implements ControlLoopRuntime {
        private final FeedforwardGains gains;

        private Runtime(FeedforwardGains gains) {
            this.gains = gains;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double target = context.target();
            double sign = Math.signum(target);
            return ControlOutput.voltage(gains.staticGain * sign + gains.velocityGain * target + gains.gravityGain);
        }
    }
}
