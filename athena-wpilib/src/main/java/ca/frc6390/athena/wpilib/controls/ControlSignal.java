package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.mechanism.core.EventBinding;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import java.time.Duration;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * A bindable boolean controller signal. Signals may be composed, filtered, or
 * transformed into stateful signals while retaining the owning gamepad's hook
 * registration.
 */
public class ControlSignal implements BooleanSupplier {
    @FunctionalInterface
    interface Evaluator {
        boolean evaluate(EventContext context);
    }

    private final ControlOwner owner;
    private final String name;
    private final Evaluator evaluator;
    private final EventBinding event;
    private HookBinding binding;
    private EventContext lastContext;
    private boolean cachedValue;

    ControlSignal(ControlOwner owner, String name, Evaluator evaluator) {
        this(owner, name, evaluator, false);
    }

    ControlSignal(ControlOwner owner, String name, Evaluator evaluator, boolean pulse) {
        this.owner = owner;
        this.name = sanitizeName(name);
        this.evaluator = Objects.requireNonNull(evaluator, "evaluator");
        this.event = new ControlEvent(this, pulse);
        this.binding = new HookBinding(event);
        if (owner != null) {
            owner.register(this);
        }
    }

    /** Returns the stable diagnostic name of this signal. */
    public final String name() {
        return name;
    }

    /** Runs once when this signal changes from false to true. */
    public ControlSignal onTrue(DeviceAction action) {
        binding = binding.onStart(action);
        return this;
    }

    /** Runs once when this signal changes from false to true. */
    public ControlSignal onTrue(ActionBinding action) {
        binding = binding.onStart(action);
        return this;
    }

    /** Runs once when this signal changes from false to true. */
    public ControlSignal onTrue(Runnable action) {
        binding = binding.onStart(action);
        return this;
    }

    /** Runs on every runtime tick for which this signal is true. */
    public ControlSignal whileTrue(DeviceAction action) {
        binding = binding.whileActive(action);
        return this;
    }

    /** Runs on every runtime tick for which this signal is true. */
    public ControlSignal whileTrue(ActionBinding action) {
        binding = binding.whileActive(action);
        return this;
    }

    /** Runs on every runtime tick for which this signal is true. */
    public ControlSignal whileTrue(Runnable action) {
        binding = binding.whileActive(action);
        return this;
    }

    /** Runs once when this signal changes from true to false. */
    public ControlSignal onFalse(DeviceAction action) {
        binding = binding.onEnd(action);
        return this;
    }

    /** Runs once when this signal changes from true to false. */
    public ControlSignal onFalse(ActionBinding action) {
        binding = binding.onEnd(action);
        return this;
    }

    /** Runs once when this signal changes from true to false. */
    public ControlSignal onFalse(Runnable action) {
        binding = binding.onEnd(action);
        return this;
    }

    /** Runs on every runtime tick for which this signal is false. */
    public ControlSignal whileFalse(DeviceAction action) {
        binding = binding.whileInactive(action);
        return this;
    }

    /** Runs on every runtime tick for which this signal is false. */
    public ControlSignal whileFalse(ActionBinding action) {
        binding = binding.whileInactive(action);
        return this;
    }

    /** Runs on every runtime tick for which this signal is false. */
    public ControlSignal whileFalse(Runnable action) {
        binding = binding.whileInactive(action);
        return this;
    }

    /** Returns a signal that is true when both inputs are true. */
    public ControlSignal and(BooleanSupplier other) {
        Objects.requireNonNull(other, "other");
        return derive("and", context -> sample(context) && value(other, context));
    }

    /** Returns a signal that is true when either input is true. */
    public ControlSignal or(BooleanSupplier other) {
        Objects.requireNonNull(other, "other");
        return derive("or", context -> sample(context) || value(other, context));
    }

    /** Returns a signal that is true when exactly one input is true. */
    public ControlSignal xor(BooleanSupplier other) {
        Objects.requireNonNull(other, "other");
        return derive("xor", context -> sample(context) ^ value(other, context));
    }

    /** Returns the logical inverse of this signal. */
    public ControlSignal negate() {
        return derive("not", context -> !sample(context));
    }

    /** Allows this signal through only while the supplied condition is true. */
    public ControlSignal onlyIf(BooleanSupplier condition) {
        return and(condition);
    }

    /** Allows this signal through only while the supplied condition is false. */
    public ControlSignal unless(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return derive("unless", context -> sample(context) && !value(condition, context));
    }

    /** Debounces both transitions by the same duration. */
    public ControlSignal debounce(Duration duration) {
        return debounce(duration, duration);
    }

    /** Debounces rising and falling transitions independently. */
    public ControlSignal debounce(Duration rising, Duration falling) {
        double risingSeconds = seconds(rising, "rising");
        double fallingSeconds = seconds(falling, "falling");
        final class State {
            boolean initialized;
            boolean output;
            boolean candidate;
            double candidateSince;

            boolean evaluate(EventContext context) {
                boolean input = sample(context);
                if (!initialized) {
                    initialized = true;
                    output = false;
                    candidate = input;
                    candidateSince = context.nowSeconds();
                    return output;
                }
                if (input != candidate) {
                    candidate = input;
                    candidateSince = context.nowSeconds();
                }
                double delay = candidate ? risingSeconds : fallingSeconds;
                if (candidate != output && context.nowSeconds() - candidateSince >= delay) {
                    output = candidate;
                }
                return output;
            }
        }
        State state = new State();
        return derive("debounce", state::evaluate);
    }

    /** Creates a false-initialized state that flips on each rising edge. */
    public ToggleSignal toggle() {
        return toggle(false);
    }

    /** Creates a state that flips on each rising edge. */
    public ToggleSignal toggle(boolean initialState) {
        return ToggleSignal.create(this, initialState);
    }

    @Override
    public boolean getAsBoolean() {
        double now = System.nanoTime() * 1.0e-9;
        return sample(new EventContext(
                now,
                0.0,
                LifecycleMode.TELEOP,
                LifecyclePhase.PERIODIC,
                true,
                false));
    }

    final boolean sample(EventContext context) {
        EventContext safeContext = context == null ? EventContext.empty() : context;
        if (lastContext != safeContext) {
            cachedValue = evaluator.evaluate(safeContext);
            lastContext = safeContext;
        }
        return cachedValue;
    }

    final HookBinding binding() {
        return binding;
    }

    final ControlOwner owner() {
        return owner;
    }

    final ControlSignal derive(String suffix, Evaluator derivedEvaluator) {
        return new ControlSignal(owner, name + "." + suffix, derivedEvaluator);
    }

    final ControlSignal pulse(String suffix, Evaluator derivedEvaluator) {
        return new ControlSignal(owner, name + "." + suffix, derivedEvaluator, true);
    }

    static boolean value(BooleanSupplier supplier, EventContext context) {
        return supplier instanceof ControlSignal signal
                ? signal.sample(context)
                : supplier.getAsBoolean();
    }

    static double seconds(Duration duration, String parameter) {
        Objects.requireNonNull(duration, parameter);
        if (duration.isNegative()) {
            throw new IllegalArgumentException(parameter + " must not be negative");
        }
        return duration.toNanos() * 1.0e-9;
    }

    private static String sanitizeName(String requested) {
        return requested == null || requested.isBlank() ? "signal" : requested;
    }

    private record ControlEvent(ControlSignal signal, boolean pulse) implements EventBinding {
        private ControlEvent {
            Objects.requireNonNull(signal, "signal");
        }

        @Override
        public String name() {
            return signal.name();
        }

        @Override
        public boolean sourceActive(EventContext context) {
            return signal.sample(context);
        }
    }
}
