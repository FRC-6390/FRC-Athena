package ca.frc6390.athena.mechanism.ref;

import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRef;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.spec.PidSpec;

/**
 * Reusable PID declaration for robot constants.
 */
public record PidRef(double p, double i, double d, double iZone, double tolerance) implements ControlLoopRef {
    public static PidRef of(double p, double i, double d) {
        return new PidRef(p, i, d, 0.0, 0.0);
    }

    public PidRef iZone(double iZone) {
        return new PidRef(p, i, d, iZone, tolerance);
    }

    public PidRef tolerance(double tolerance) {
        return new PidRef(p, i, d, iZone, tolerance);
    }

    public PidSpec toSpec() {
        return new PidSpec(p, i, d, iZone, tolerance);
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(this);
    }

    private static final class Runtime implements ControlLoopRuntime {
        private final PidRef ref;
        private double integral;
        private double previousError;
        private boolean first = true;

        private Runtime(PidRef ref) {
            this.ref = ref;
        }

        @Override
        public void reset(ControlLoopContext context) {
            integral = 0.0;
            previousError = 0.0;
            first = true;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double error = context.target() - context.position();
            double dt = Math.max(context.dtSeconds(), 1.0e-9);
            if (ref.iZone <= 0.0 || Math.abs(error) <= ref.iZone) {
                integral += error * dt;
            } else {
                integral = 0.0;
            }
            double derivative = first ? 0.0 : (error - previousError) / dt;
            first = false;
            previousError = error;
            return ControlOutput.percent(ref.p * error + ref.i * integral + ref.d * derivative);
        }
    }
}
