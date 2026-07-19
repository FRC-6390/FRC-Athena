package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/** A physical or processed controller button with bindable gestures. */
public class ButtonSignal extends ControlSignal {
    private static final Duration DEFAULT_CLICK_WINDOW = Duration.ofMillis(300);

    ButtonSignal(String name, BooleanSupplier source) {
        this(null, name, context -> source.getAsBoolean());
        Objects.requireNonNull(source, "source");
    }

    ButtonSignal(ControlOwner owner, String name, Evaluator evaluator) {
        super(owner, name, evaluator);
    }

    /** Runs once when the button is pressed. */
    public ButtonSignal onPress(DeviceAction action) {
        super.onTrue(action);
        return this;
    }

    /** Runs once when the button is pressed. */
    public ButtonSignal onPress(ActionBinding action) {
        super.onTrue(action);
        return this;
    }

    /** Runs once when the button is pressed. */
    public ButtonSignal onPress(Runnable action) {
        super.onTrue(action);
        return this;
    }

    /** Runs on every runtime tick while the button is held. */
    public ButtonSignal whileHeld(DeviceAction action) {
        super.whileTrue(action);
        return this;
    }

    /** Runs on every runtime tick while the button is held. */
    public ButtonSignal whileHeld(ActionBinding action) {
        super.whileTrue(action);
        return this;
    }

    /** Runs on every runtime tick while the button is held. */
    public ButtonSignal whileHeld(Runnable action) {
        super.whileTrue(action);
        return this;
    }

    /** Runs once when the button is released. */
    public ButtonSignal onRelease(DeviceAction action) {
        super.onFalse(action);
        return this;
    }

    /** Runs once when the button is released. */
    public ButtonSignal onRelease(ActionBinding action) {
        super.onFalse(action);
        return this;
    }

    /** Runs once when the button is released. */
    public ButtonSignal onRelease(Runnable action) {
        super.onFalse(action);
        return this;
    }

    /** Returns a one-tick pulse when the button is pressed. */
    public ControlSignal pressed() {
        final class State {
            boolean previous;

            boolean evaluate(EventContext context) {
                boolean current = sample(context);
                boolean result = current && !previous;
                previous = current;
                return result;
            }
        }
        State state = new State();
        return pulse("pressed", state::evaluate);
    }

    /** Returns a one-tick pulse when the button is released. */
    public ControlSignal released() {
        final class State {
            boolean previous;

            boolean evaluate(EventContext context) {
                boolean current = sample(context);
                boolean result = !current && previous;
                previous = current;
                return result;
            }
        }
        State state = new State();
        return pulse("released", state::evaluate);
    }

    /** Returns a level that becomes true after the button is held for the duration. */
    public ControlSignal heldFor(Duration duration) {
        double threshold = seconds(duration, "duration");
        final class State {
            boolean previous;
            double pressedAt;

            boolean evaluate(EventContext context) {
                boolean current = sample(context);
                if (current && !previous) {
                    pressedAt = context.nowSeconds();
                }
                previous = current;
                return current && context.nowSeconds() - pressedAt >= threshold;
            }
        }
        State state = new State();
        return derive("heldFor[" + duration.toMillis() + "ms]", state::evaluate);
    }

    /** Returns a one-tick pulse when the hold duration is first reached. */
    public ControlSignal holdStarted(Duration duration) {
        ControlSignal held = heldFor(duration);
        final class State {
            boolean previous;

            boolean evaluate(EventContext context) {
                boolean current = held.sample(context);
                boolean result = current && !previous;
                previous = current;
                return result;
            }
        }
        State state = new State();
        return pulse("holdStarted[" + duration.toMillis() + "ms]", state::evaluate);
    }

    /** Pulses on release when the button was held no longer than the maximum duration. */
    public ControlSignal shortPress(Duration maximum) {
        double maximumSeconds = seconds(maximum, "maximum");
        final class State {
            boolean previous;
            double pressedAt;

            boolean evaluate(EventContext context) {
                boolean current = sample(context);
                if (current && !previous) {
                    pressedAt = context.nowSeconds();
                }
                boolean result = !current && previous
                        && context.nowSeconds() - pressedAt <= maximumSeconds;
                previous = current;
                return result;
            }
        }
        State state = new State();
        return pulse("shortPress[" + maximum.toMillis() + "ms]", state::evaluate);
    }

    /** Pulses immediately on press, then repeatedly after the delay at the interval. */
    public ControlSignal repeated(Duration delay, Duration interval) {
        double delaySeconds = seconds(delay, "delay");
        double intervalSeconds = seconds(interval, "interval");
        if (intervalSeconds <= 0.0) {
            throw new IllegalArgumentException("interval must be greater than zero");
        }
        final class State {
            boolean previous;
            double nextRepeat;

            boolean evaluate(EventContext context) {
                boolean current = sample(context);
                boolean result = false;
                if (current && !previous) {
                    result = true;
                    nextRepeat = context.nowSeconds() + delaySeconds;
                } else if (current && context.nowSeconds() >= nextRepeat) {
                    result = true;
                    nextRepeat = context.nowSeconds() + intervalSeconds;
                }
                previous = current;
                return result;
            }
        }
        State state = new State();
        return pulse("repeat", state::evaluate);
    }

    /** Emits after one completed click and the default multi-click window. */
    public ControlSignal click() {
        return clicks(1);
    }

    /** Emits after two completed clicks and the default multi-click window. */
    public ControlSignal doubleClick() {
        return clicks(2);
    }

    /** Emits after the exact number of clicks and the default multi-click window. */
    public ControlSignal clicks(int count) {
        return clicks(DEFAULT_CLICK_WINDOW).exactly(count);
    }

    /** Emits after the exact number of clicks and the supplied inter-click window. */
    public ControlSignal clicks(int count, Duration window) {
        return clicks(window).exactly(count);
    }

    /** Creates a shared recognizer for several exact or ranged click bindings. */
    public ClickSequence clicks(Duration window) {
        return new ClickSequence(this, window);
    }

    @Override
    public ButtonSignal debounce(Duration duration) {
        return debounce(duration, duration);
    }

    @Override
    public ButtonSignal debounce(Duration rising, Duration falling) {
        ControlSignal processed = super.debounce(rising, falling);
        return new ButtonSignal(owner(), processed.name(), processed::sample);
    }

    // Compatibility aliases retained for existing robot projects.
    public ButtonSignal onActive(DeviceAction action) { return onPress(action); }
    public ButtonSignal onActive(ActionBinding action) { return onPress(action); }
    public ButtonSignal onActive(Runnable action) { return onPress(action); }
    public ButtonSignal whileActive(DeviceAction action) { return whileHeld(action); }
    public ButtonSignal whileActive(ActionBinding action) { return whileHeld(action); }
    public ButtonSignal whileActive(Runnable action) { return whileHeld(action); }
    public ButtonSignal onDeactive(DeviceAction action) { return onRelease(action); }
    public ButtonSignal onDeactive(ActionBinding action) { return onRelease(action); }
    public ButtonSignal onDeactive(Runnable action) { return onRelease(action); }
    public ButtonSignal whileDeactive(DeviceAction action) { super.whileFalse(action); return this; }
    public ButtonSignal whileDeactive(ActionBinding action) { super.whileFalse(action); return this; }
    public ButtonSignal whileDeactive(Runnable action) { super.whileFalse(action); return this; }
}
