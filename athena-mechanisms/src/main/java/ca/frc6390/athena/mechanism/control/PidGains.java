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
public record PidGains(double p, double i, double d, double iZone) implements ControlLoop {
    private static final double MAX_OUTPUT_VOLTS = 12.0;

    public PidGains {
        requireFinite(p, "PID proportional gain");
        requireFinite(i, "PID integral gain");
        requireFinite(d, "PID derivative gain");
        if (!Double.isFinite(iZone) || iZone < 0.0) {
            throw new IllegalArgumentException("PID integral zone must be finite and non-negative.");
        }
    }

    public static PidGains of(double p, double i, double d) {
        return new PidGains(p, i, d, 0.0);
    }

    /**
     * Limits integral accumulation to errors within the supplied magnitude.
     *
     * <p>The zone uses the configured feedback units. A zero zone allows integration
     * at every finite error.</p>
     *
     * @param iZone maximum absolute error that permits integration
     * @return updated gains
     */
    public PidGains iZone(double iZone) {
        return new PidGains(p, i, d, iZone);
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
        private final PidGains gains;
        private double integral;
        private double previousMeasurement;
        private boolean first = true;

        private Runtime(PidGains gains) {
            this.gains = gains;
        }

        @Override
        public void reset(ControlLoopContext context) {
            integral = 0.0;
            previousMeasurement = 0.0;
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
            double dt = Double.isFinite(context.dtSeconds()) && context.dtSeconds() > 0.0
                    ? context.dtSeconds()
                    : 0.0;
            double derivative = first || dt == 0.0 ? 0.0 : -(measurement - previousMeasurement) / dt;
            if (dt > 0.0 && (gains.iZone <= 0.0 || Math.abs(error) <= gains.iZone)) {
                integral = antiWindupIntegral(integral + error * dt, error, derivative);
            } else if (gains.iZone > 0.0 && Math.abs(error) > gains.iZone) {
                integral = 0.0;
            }
            first = false;
            previousMeasurement = measurement;
            return ControlOutput.voltage(gains.p * error + gains.i * integral + gains.d * derivative);
        }

        @Override
        public void applied(ControlLoopContext context, Output requested, Output applied) {
            if (gains.i == 0.0
                    || !(requested instanceof Output.Voltage requestedVoltage)
                    || !(applied instanceof Output.Voltage appliedVoltage)) {
                return;
            }
            double excessVolts = requestedVoltage.volts() - appliedVoltage.volts();
            double integralVolts = gains.i * integral;
            if (excessVolts > 0.0 && integralVolts > 0.0) {
                integral -= Math.min(excessVolts, integralVolts) / gains.i;
            } else if (excessVolts < 0.0 && integralVolts < 0.0) {
                integral -= Math.max(excessVolts, integralVolts) / gains.i;
            }
        }

        private double antiWindupIntegral(double candidate, double error, double derivative) {
            if (gains.i == 0.0) {
                return 0.0;
            }
            double nonIntegralVolts = gains.p * error + gains.d * derivative;
            double currentVolts = nonIntegralVolts + gains.i * integral;
            double candidateVolts = nonIntegralVolts + gains.i * candidate;
            double integralDeltaVolts = gains.i * (candidate - integral);
            if (candidateVolts > MAX_OUTPUT_VOLTS && integralDeltaVolts > 0.0) {
                return currentVolts < MAX_OUTPUT_VOLTS
                        ? integral + (MAX_OUTPUT_VOLTS - currentVolts) / gains.i
                        : integral;
            }
            if (candidateVolts < -MAX_OUTPUT_VOLTS && integralDeltaVolts < 0.0) {
                return currentVolts > -MAX_OUTPUT_VOLTS
                        ? integral + (-MAX_OUTPUT_VOLTS - currentVolts) / gains.i
                        : integral;
            }
            return candidate;
        }
    }
}
