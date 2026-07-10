package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * A controller button signal with bindings owned by its gamepad.
 */
public final class ButtonSignal implements BooleanSupplier {
    private final String name;
    private final BooleanSupplier source;
    private HookBinding binding;

    ButtonSignal(String name, BooleanSupplier source) {
        this.name = Objects.requireNonNull(name, "name");
        this.source = Objects.requireNonNull(source, "source");
        this.binding = new HookBinding(Events.when(source).active());
    }

    public ButtonSignal onActive(DeviceAction action) {
        binding = binding.onStart(action);
        return this;
    }

    public ButtonSignal onActive(Runnable action) {
        binding = binding.onStart(action);
        return this;
    }

    public ButtonSignal whileActive(DeviceAction action) {
        binding = binding.whileActive(action);
        return this;
    }

    public ButtonSignal whileActive(Runnable action) {
        binding = binding.whileActive(action);
        return this;
    }

    public ButtonSignal onDeactive(DeviceAction action) {
        binding = binding.onEnd(action);
        return this;
    }

    public ButtonSignal onDeactive(Runnable action) {
        binding = binding.onEnd(action);
        return this;
    }

    public ButtonSignal whileDeactive(DeviceAction action) {
        binding = binding.whileInactive(action);
        return this;
    }

    public ButtonSignal whileDeactive(Runnable action) {
        binding = binding.whileInactive(action);
        return this;
    }

    @Override
    public boolean getAsBoolean() {
        return source.getAsBoolean();
    }

    String name() {
        return name;
    }

    HookBinding binding() {
        return binding;
    }
}
