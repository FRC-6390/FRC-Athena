package ca.frc6390.athena.mechanism.ref;

import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRef;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.spec.FeedforwardSpec;

/**
 * Reusable feedforward declaration for robot constants.
 */
public record FeedforwardRef(double staticGain, double velocityGain, double gravityGain) implements ControlLoopRef {
    public static FeedforwardRef of(double staticGain, double velocityGain, double gravityGain) {
        return new FeedforwardRef(staticGain, velocityGain, gravityGain);
    }

    public static FeedforwardRef simple(double staticGain, double velocityGain, double gravityGain) {
        return of(staticGain, velocityGain, gravityGain);
    }

    public FeedforwardSpec toSpec() {
        return new FeedforwardSpec(staticGain, velocityGain, gravityGain);
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(this);
    }

    private static final class Runtime implements ControlLoopRuntime {
        private final FeedforwardRef ref;

        private Runtime(FeedforwardRef ref) {
            this.ref = ref;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double target = context.target();
            double sign = Math.signum(target);
            return ControlOutput.voltage(ref.staticGain * sign + ref.velocityGain * target + ref.gravityGain);
        }
    }
}
