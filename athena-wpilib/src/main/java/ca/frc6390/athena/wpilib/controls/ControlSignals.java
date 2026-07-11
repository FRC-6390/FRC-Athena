package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import java.util.Arrays;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/** Factories for controller conditions and multi-button gestures. */
public final class ControlSignals {
    private ControlSignals() {
    }

    /** Returns a signal active only when every supplied condition is true. */
    public static ControlSignal allOf(BooleanSupplier... signals) {
        BooleanSupplier[] safeSignals = copy(signals);
        return new ControlSignal(firstOwner(safeSignals), "allOf", context -> {
            for (BooleanSupplier signal : safeSignals) {
                if (!ControlSignal.value(signal, context)) {
                    return false;
                }
            }
            return true;
        });
    }

    /** Returns a signal active when any supplied condition is true. */
    public static ControlSignal anyOf(BooleanSupplier... signals) {
        BooleanSupplier[] safeSignals = copy(signals);
        return new ControlSignal(firstOwner(safeSignals), "anyOf", context -> {
            for (BooleanSupplier signal : safeSignals) {
                if (ControlSignal.value(signal, context)) {
                    return true;
                }
            }
            return false;
        });
    }

    /** Returns a signal active only when every supplied condition is false. */
    public static ControlSignal noneOf(BooleanSupplier... signals) {
        return anyOf(signals).negate();
    }

    /** Creates a chord that pulses when every button is pressed within the supplied window. */
    public static Chord chord(ButtonSignal... buttons) {
        return new Chord(buttons);
    }

    /** Creates an ordered pulse sequence that must complete within a duration. */
    public static Sequence sequence(ControlSignal... pulses) {
        return new Sequence(pulses);
    }

    /** Builder for a timed multi-button chord. */
    public static final class Chord {
        private final ButtonSignal[] buttons;

        private Chord(ButtonSignal[] buttons) {
            this.buttons = copyButtons(buttons);
        }

        /** Emits when all chord buttons become held within the duration. */
        public ControlSignal within(Duration duration) {
            double window = ControlSignal.seconds(duration, "duration");
            Gamepad owner = buttons[0].owner();
            final class State {
                final double[] pressedAt = new double[buttons.length];
                final boolean[] previous = new boolean[buttons.length];
                boolean previousComplete;

                boolean evaluate(EventContext context) {
                    boolean complete = true;
                    double earliest = Double.POSITIVE_INFINITY;
                    double latest = Double.NEGATIVE_INFINITY;
                    for (int index = 0; index < buttons.length; index++) {
                        boolean active = buttons[index].sample(context);
                        if (active && !previous[index]) {
                            pressedAt[index] = context.nowSeconds();
                        }
                        previous[index] = active;
                        complete &= active;
                        earliest = Math.min(earliest, pressedAt[index]);
                        latest = Math.max(latest, pressedAt[index]);
                    }
                    boolean valid = complete && latest - earliest <= window;
                    boolean emitted = valid && !previousComplete;
                    previousComplete = complete;
                    return emitted;
                }
            }
            State state = new State();
            return new ControlSignal(owner, "chord", state::evaluate, true);
        }
    }

    /** Builder for an ordered sequence of pulse signals. */
    public static final class Sequence {
        private final ControlSignal[] pulses;

        private Sequence(ControlSignal[] pulses) {
            Objects.requireNonNull(pulses, "pulses");
            if (pulses.length < 2) {
                throw new IllegalArgumentException("a sequence requires at least two signals");
            }
            this.pulses = Arrays.copyOf(pulses, pulses.length);
            for (ControlSignal pulse : this.pulses) {
                Objects.requireNonNull(pulse, "pulse");
            }
        }

        /** Emits when the ordered sequence completes before the duration expires. */
        public ControlSignal within(Duration duration) {
            double window = ControlSignal.seconds(duration, "duration");
            final class State {
                int next;
                double startedAt;

                boolean evaluate(EventContext context) {
                    if (next > 0 && context.nowSeconds() - startedAt > window) {
                        next = 0;
                    }
                    boolean expected = false;
                    for (int index = 0; index < pulses.length; index++) {
                        boolean active = pulses[index].sample(context);
                        if (index == next) {
                            expected = active;
                        }
                    }
                    if (!expected) {
                        return false;
                    }
                    if (next == 0) {
                        startedAt = context.nowSeconds();
                    }
                    next++;
                    if (next == pulses.length) {
                        next = 0;
                        return true;
                    }
                    return false;
                }
            }
            State state = new State();
            return new ControlSignal(pulses[0].owner(), "sequence", state::evaluate, true);
        }
    }

    private static BooleanSupplier[] copy(BooleanSupplier[] signals) {
        Objects.requireNonNull(signals, "signals");
        BooleanSupplier[] copy = Arrays.copyOf(signals, signals.length);
        for (BooleanSupplier signal : copy) {
            Objects.requireNonNull(signal, "signal");
        }
        return copy;
    }

    private static ButtonSignal[] copyButtons(ButtonSignal[] buttons) {
        Objects.requireNonNull(buttons, "buttons");
        if (buttons.length < 2) {
            throw new IllegalArgumentException("a chord requires at least two buttons");
        }
        ButtonSignal[] copy = Arrays.copyOf(buttons, buttons.length);
        for (ButtonSignal button : copy) {
            Objects.requireNonNull(button, "button");
        }
        return copy;
    }

    private static Gamepad firstOwner(BooleanSupplier[] signals) {
        for (BooleanSupplier supplier : signals) {
            if (supplier instanceof ControlSignal signal && signal.owner() != null) {
                return signal.owner();
            }
        }
        return null;
    }
}
