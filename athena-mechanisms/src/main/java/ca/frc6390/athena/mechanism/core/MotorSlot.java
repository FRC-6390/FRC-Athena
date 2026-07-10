package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.UnaryOperator;

/**
 * Fillable motor slot.
 *
 * @param <O> owner type
 */
public final class MotorSlot<O> extends Slot<O, MotorDevice> {
    private UnaryOperator<MotorDevice> configuration = UnaryOperator.identity();

    MotorSlot(O owner, String name, Runnable onFill) {
        super(owner, name, onFill);
    }

    @Override
    public O fill(MotorDevice value) {
        return super.fill(configuration.apply(Objects.requireNonNull(value, name())));
    }

    public MotorSlot<O> configure(UnaryOperator<MotorDevice> configure) {
        UnaryOperator<MotorDevice> next = Objects.requireNonNull(configure, "configure");
        configuration = configuration.andThen(next)::apply;
        if (filled()) {
            super.fill(next.apply(get()));
        }
        return this;
    }

    public MotorSlot<O> brake() {
        return configure(MotorDevice::brake);
    }

    public MotorSlot<O> coast() {
        return configure(MotorDevice::coast);
    }

    public MotorSlot<O> neutralMode(MotorNeutralMode neutralMode) {
        return configure(motor -> motor.neutralMode(neutralMode));
    }

    public MotorSlot<O> currentLimit(int amps) {
        return configure(motor -> motor.currentLimit(amps));
    }

    public MotorSlot<O> inverted() {
        return inverted(true);
    }

    public MotorSlot<O> inverted(boolean inverted) {
        return configure(motor -> motor.inverted(inverted));
    }

    public <T> MotorSlot<O> vendor(Class<T> optionType, Consumer<T> configure) {
        return configure(motor -> motor.vendor(optionType, configure));
    }
}
