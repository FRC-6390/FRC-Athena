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

/** PID gains that are automatically exposed for live editing when declared by a mechanism. */
public final class PidGains implements ControlLoop, TelemetrySource {
    private static final double MAX_OUTPUT_VOLTS = 12.0;
    private volatile double p;
    private volatile double i;
    private volatile double d;
    private volatile double iZone;
    private volatile boolean disabled;
    private final Map<String, TelemetryValue> telemetry = Map.of(
            "p", TelemetryValue.writableNumber(this::p, this::setP),
            "i", TelemetryValue.writableNumber(this::i, this::setI),
            "d", TelemetryValue.writableNumber(this::d, this::setD),
            "iZone", TelemetryValue.writableNumber(this::iZone, this::setIZone),
            "disabled", TelemetryValue.writableBoolean(this::isDisabled, value -> disabled = value));

    public PidGains(double p, double i, double d, double iZone) {
        setP(p); setI(i); setD(d); setIZone(iZone);
    }

    public static PidGains of(double p, double i, double d) { return new PidGains(p, i, d, 0.0); }
    public double p() { return p; }
    public double i() { return i; }
    public double d() { return d; }
    public double iZone() { return iZone; }
    public boolean isDisabled() { return disabled; }

    public PidGains iZone(double value) { return new PidGains(p, i, d, value); }

    /** Creates independent PID state backed by these live-editable gains. */
    public PidController controller() { return new PidController(this); }

    @Override public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(binding.control().continuousRange());
    }
    @Override public ControlLoopRole role() { return ControlLoopRole.DEVICE_CONFIGURABLE; }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        return telemetry;
    }

    private void setP(double value) { p = finite(value, "PID proportional gain"); }
    private void setI(double value) { i = finite(value, "PID integral gain"); }
    private void setD(double value) { d = finite(value, "PID derivative gain"); }
    private void setIZone(double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException("PID integral zone must be finite and non-negative.");
        }
        iZone = value;
    }
    private static double finite(double value, String description) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(description + " must be finite.");
        return value;
    }

    @Override public boolean equals(Object other) {
        return other instanceof PidGains gains
                && Double.compare(p, gains.p) == 0 && Double.compare(i, gains.i) == 0
                && Double.compare(d, gains.d) == 0 && Double.compare(iZone, gains.iZone) == 0;
    }
    @Override public int hashCode() { return Objects.hash(p, i, d, iZone); }
    @Override public String toString() {
        return "PidGains[p=" + p + ", i=" + i + ", d=" + d + ", iZone=" + iZone + "]";
    }

    private final class Runtime implements ControlLoopRuntime {
        private final PidController controller;

        private Runtime(ca.frc6390.athena.hardware.device.Range continuousRange) {
            PidController configured = PidGains.this.controller()
                    .outputRange(-MAX_OUTPUT_VOLTS, MAX_OUTPUT_VOLTS);
            controller = continuousRange == null
                    ? configured
                    : configured.continuous(continuousRange.minimum(), continuousRange.maximum());
        }

        @Override public void reset(ControlLoopContext context) {
            controller.reset();
        }

        @Override public ControlOutput calculate(ControlLoopContext context) {
            double measurement = context.request() instanceof Output.Velocity ? context.velocity() : context.position();
            if (!Double.isFinite(measurement) || !Double.isFinite(context.target())) {
                return ControlOutput.neutral();
            }
            return ControlOutput.voltage(controller.calculate(
                    measurement, context.target(), context.dtSeconds()));
        }

        @Override public ControlLoopTrace trace() {
            return new ControlLoopTrace(
                    controller.proportionalOutput(),
                    controller.integralOutput(),
                    controller.derivativeOutput(),
                    0, 0, 0, 0);
        }

        @Override public void applied(ControlLoopContext context, Output requested, Output applied) {
            if (!(requested instanceof Output.Voltage requestedVoltage)
                    || !(applied instanceof Output.Voltage appliedVoltage)) return;
            controller.applied(requestedVoltage.volts(), appliedVoltage.volts());
        }
    }
}
