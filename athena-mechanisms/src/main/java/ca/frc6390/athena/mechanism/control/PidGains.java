package ca.frc6390.athena.mechanism.control;

import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopRole;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Output;

/**
 * Reusable PID gains whose calculated effort is always expressed in volts.
 */
public record PidGains(double p, double i, double d, double iZone, double tolerance) implements ControlLoop {
    public static PidGains of(double p, double i, double d) {
        return new PidGains(p, i, d, 0.0, 0.0);
    }

    public PidGains iZone(double iZone) {
        return new PidGains(p, i, d, iZone, tolerance);
    }

    public PidGains tolerance(double tolerance) {
        return new PidGains(p, i, d, iZone, tolerance);
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
        private final PidGains gains;
        private double integral;
        private double previousError;
        private boolean first = true;

        private Runtime(PidGains gains) {
            this.gains = gains;
        }

        @Override
        public void reset(ControlLoopContext context) {
            integral = 0.0;
            previousError = 0.0;
            first = true;
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double measurement = context.request() instanceof Output.Velocity
                    ? context.velocity()
                    : context.position();
            double error = context.target() - measurement;
            if (!Double.isFinite(error)) {
                return ControlOutput.neutral();
            }
            double dt = Math.max(context.dtSeconds(), 1.0e-9);
            if (gains.iZone <= 0.0 || Math.abs(error) <= gains.iZone) {
                integral += error * dt;
            } else {
                integral = 0.0;
            }
            double derivative = first ? 0.0 : (error - previousError) / dt;
            first = false;
            previousError = error;
            return ControlOutput.voltage(gains.p * error + gains.i * integral + gains.d * derivative);
        }
    }
}
