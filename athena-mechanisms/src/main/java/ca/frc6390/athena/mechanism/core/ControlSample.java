package ca.frc6390.athena.mechanism.core;

/**
 * Latest per-cycle sample of one control binding: the target, the motion-profile reference, the tracking error and
 * the applied output in volts. The output path records it once per cycle and telemetry reads it, so both sides share
 * one object and recording allocates nothing. Every field is NaN while the control is not driven.
 */
final class ControlSample {
    /**
     * Latest sample of each driven control, so a check declared on the binding (which has no runtime
     * reference) can see the goal the runtime is driving. Written once per control when the runtime first
     * samples it; read under the lock, which is uncontended on the robot loop.
     */
    private static final java.util.Map<ControlBinding<?>, ControlSample> LATEST = new java.util.IdentityHashMap<>();

    static synchronized void publish(ControlBinding<?> control, ControlSample sample) {
        LATEST.put(control, sample);
    }

    static synchronized ControlSample latest(ControlBinding<?> control) {
        return LATEST.get(control);
    }

    private double target = Double.NaN;
    private double referencePosition = Double.NaN;
    private double referenceVelocity = Double.NaN;
    private double error = Double.NaN;
    private double outputVolts = Double.NaN;

    /** Stores this cycle's sample. Values are in the control's own units; the output is in volts. */
    void record(double target, double referencePosition, double referenceVelocity, double error, double outputVolts) {
        this.target = target;
        this.referencePosition = referencePosition;
        this.referenceVelocity = referenceVelocity;
        this.error = error;
        this.outputVolts = outputVolts;
    }

    /** Forgets the sample, for a control that is not driven this cycle. */
    void clear() {
        record(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    double target() { return target; }

    double referencePosition() { return referencePosition; }

    double referenceVelocity() { return referenceVelocity; }

    double error() { return error; }

    double output() { return outputVolts; }
}
