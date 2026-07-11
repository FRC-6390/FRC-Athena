package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/** A bindable boolean state that flips on each rising edge of its source. */
public final class ToggleSignal extends ControlSignal {
    private final State state;

    private ToggleSignal(ControlSignal source, boolean initialState, State state) {
        super(source.owner(), source.name() + ".toggle", state::evaluate);
        this.state = state;
        state.source = source;
        state.value = initialState;
        state.initialValue = initialState;
    }

    static ToggleSignal create(ControlSignal source, boolean initialState) {
        Objects.requireNonNull(source, "source");
        State state = new State();
        return new ToggleSignal(source, initialState, state);
    }

    /** Returns the currently latched state. */
    public boolean state() {
        return state.value;
    }

    /** Sets the latched state directly. */
    public ToggleSignal set(boolean enabled) {
        state.value = enabled;
        return this;
    }

    /** Sets the latched state to true. */
    public ToggleSignal enable() {
        return set(true);
    }

    /** Sets the latched state to false. */
    public ToggleSignal disable() {
        return set(false);
    }

    /** Flips the latched state directly. */
    public ToggleSignal flip() {
        state.value = !state.value;
        return this;
    }

    /** Sets the state to its initial value whenever the supplied signal is true. */
    public ToggleSignal resetWhen(BooleanSupplier signal) {
        state.resets.add(Objects.requireNonNull(signal, "signal"));
        return this;
    }

    /** Sets the state to true whenever the supplied signal is true. */
    public ToggleSignal setWhen(BooleanSupplier signal) {
        state.sets.add(Objects.requireNonNull(signal, "signal"));
        return this;
    }

    /** Sets the state to false whenever the supplied signal is true. */
    public ToggleSignal clearWhen(BooleanSupplier signal) {
        state.clears.add(Objects.requireNonNull(signal, "signal"));
        return this;
    }

    /** Controls whether disabled runtime contexts reset this state. Defaults to true. */
    public ToggleSignal resetOnDisable(boolean enabled) {
        state.resetOnDisable = enabled;
        return this;
    }

    /** Controls whether controller disconnection resets this state. Defaults to true. */
    public ToggleSignal resetOnDisconnect(boolean enabled) {
        state.resetOnDisconnect = enabled;
        return this;
    }

    private static final class State {
        private ControlSignal source;
        private final List<BooleanSupplier> resets = new ArrayList<>();
        private final List<BooleanSupplier> sets = new ArrayList<>();
        private final List<BooleanSupplier> clears = new ArrayList<>();
        private boolean value;
        private boolean initialValue;
        private boolean previousSource;
        private boolean resetOnDisable = true;
        private boolean resetOnDisconnect = true;

        private boolean evaluate(EventContext context) {
            boolean active = source.sample(context);
            boolean disconnected = source.owner() != null && !source.owner().connected();
            if (resetOnDisable && !context.enabled() || resetOnDisconnect && disconnected) {
                value = initialValue;
            } else if (active && !previousSource) {
                value = !value;
            }
            previousSource = active;

            if (any(resets, context)) {
                value = initialValue;
            }
            if (any(sets, context)) {
                value = true;
            }
            if (any(clears, context)) {
                value = false;
            }
            return value;
        }

        private static boolean any(List<BooleanSupplier> signals, EventContext context) {
            for (BooleanSupplier signal : signals) {
                if (ControlSignal.value(signal, context)) {
                    return true;
                }
            }
            return false;
        }
    }
}
