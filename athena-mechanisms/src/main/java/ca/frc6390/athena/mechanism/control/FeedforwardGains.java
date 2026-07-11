package ca.frc6390.athena.mechanism.control;

import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopRole;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;

/**
 * Reusable feedforward gains whose calculated effort is expressed in volts.
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

    @Override
    public ControlLoopRole role() {
        return ControlLoopRole.DEVICE_CONFIGURABLE;
    }

    private static final class Runtime implements ControlLoopRuntime {
        private final FeedforwardGains gains;

        private Runtime(FeedforwardGains gains) {
            this.gains = gains;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double velocity = context.reference().velocity();
            if (!Double.isFinite(velocity)) {
                return ControlOutput.neutral();
            }
            double sign = Math.signum(velocity);
            return ControlOutput.voltage(
                    gains.staticGain * sign + gains.velocityGain * velocity + gains.gravityGain);
        }
    }
}
