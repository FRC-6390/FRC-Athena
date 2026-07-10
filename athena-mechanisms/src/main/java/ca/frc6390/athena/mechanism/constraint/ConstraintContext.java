package ca.frc6390.athena.mechanism.constraint;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import java.util.Objects;

/**
 * Current and requested values supplied to a constraint.
 *
 * @param current current mechanism value
 * @param requested requested mechanism value
 * @param runtime current mechanism runtime context
 * @param <T> constrained value type
 */
public record ConstraintContext<T>(
        T current,
        T requested,
        MechanismContext runtime,
        ActionContext hardware) {
    public ConstraintContext(T current, T requested, MechanismContext runtime) {
        this(current, requested, runtime, ActionContext.empty());
    }

    public ConstraintContext {
        Objects.requireNonNull(current, "current");
        Objects.requireNonNull(requested, "requested");
        runtime = runtime == null ? MechanismContext.empty() : runtime;
        hardware = hardware == null ? ActionContext.empty() : hardware;
    }

    public ConstraintContext<T> requested(T value) {
        return new ConstraintContext<>(current, Objects.requireNonNull(value, "value"), runtime, hardware);
    }

    public double position(PositionSignal signal) {
        return Objects.requireNonNull(signal, "signal").position(hardware);
    }

    public double velocity(VelocitySignal signal) {
        return Objects.requireNonNull(signal, "signal").velocity(hardware);
    }
}
