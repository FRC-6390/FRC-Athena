package ca.frc6390.athena.robot.explore;

import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Rates for random controller input. {@link #generate} turns a seed, a set of ports and a duration into a
 * time-ordered list of {@link InputEvent}s. Every port has its own generator, so adding a port does not change
 * the events of the others.
 *
 * <p>Buttons press at a Poisson rate, hold for an exponential time, and never overlap with themselves. Axes
 * random-walk on a fixed update grid. POVs change at a Poisson rate.</p>
 */
public final class InputModel {
    private final int buttons;
    private final double buttonPressesPerSecond;
    private final double meanHoldSeconds;
    private final int axes;
    private final double axisUpdatesPerSecond;
    private final double axisStep;
    private final double povChangesPerSecond;

    private InputModel(int buttons, double buttonPressesPerSecond, double meanHoldSeconds, int axes,
            double axisUpdatesPerSecond, double axisStep, double povChangesPerSecond) {
        if (buttons < 0 || buttons > 64) throw new IllegalArgumentException("buttons must be 0..64");
        if (axes < 0 || axes > RuntimeInputSnapshot.AXES) {
            throw new IllegalArgumentException("axes must be 0.." + RuntimeInputSnapshot.AXES);
        }
        this.buttons = buttons;
        this.buttonPressesPerSecond = nonNegative(buttonPressesPerSecond, "buttonPressesPerSecond");
        this.meanHoldSeconds = positive(meanHoldSeconds, "meanHoldSeconds");
        this.axes = axes;
        this.axisUpdatesPerSecond = nonNegative(axisUpdatesPerSecond, "axisUpdatesPerSecond");
        this.axisStep = nonNegative(axisStep, "axisStep");
        this.povChangesPerSecond = nonNegative(povChangesPerSecond, "povChangesPerSecond");
    }

    /** 8 buttons pressed about 0.6 times a second and held 1.2 s on average; 4 axes at 4 Hz; POVs rarely. */
    public static InputModel standard() {
        return new InputModel(8, 0.6, 1.2, 4, 4.0, 0.25, 0.1);
    }

    /** No input at all. */
    public static InputModel none() {
        return new InputModel(0, 0.0, 1.0, 0, 0.0, 0.0, 0.0);
    }

    /** Sets the number of buttons (one-based 1..n) the model presses. */
    public InputModel buttons(int count) {
        return new InputModel(count, buttonPressesPerSecond, meanHoldSeconds, axes, axisUpdatesPerSecond,
                axisStep, povChangesPerSecond);
    }

    /** Sets the Poisson rate of new button presses per port. */
    public InputModel buttonPressesPerSecond(double rate) {
        return new InputModel(buttons, rate, meanHoldSeconds, axes, axisUpdatesPerSecond, axisStep,
                povChangesPerSecond);
    }

    /** Sets the mean hold time of a button press. */
    public InputModel meanHoldSeconds(double seconds) {
        return new InputModel(buttons, buttonPressesPerSecond, seconds, axes, axisUpdatesPerSecond, axisStep,
                povChangesPerSecond);
    }

    /** Sets the number of axes (0..6) that random-walk. */
    public InputModel axes(int count) {
        return new InputModel(buttons, buttonPressesPerSecond, meanHoldSeconds, count, axisUpdatesPerSecond,
                axisStep, povChangesPerSecond);
    }

    /** Sets the axis update rate and the standard deviation of each step. */
    public InputModel axisWalk(double updatesPerSecond, double stepStandardDeviation) {
        return new InputModel(buttons, buttonPressesPerSecond, meanHoldSeconds, axes, updatesPerSecond,
                stepStandardDeviation, povChangesPerSecond);
    }

    /** Sets the Poisson rate of POV changes per port. */
    public InputModel povChangesPerSecond(double rate) {
        return new InputModel(buttons, buttonPressesPerSecond, meanHoldSeconds, axes, axisUpdatesPerSecond,
                axisStep, rate);
    }

    /**
     * Generates the input script for a run.
     *
     * @param seed run seed
     * @param ports zero-based controller ports
     * @param durationSeconds run length
     * @return events sorted by time, ties kept in generation order
     */
    public List<InputEvent> generate(long seed, int[] ports, double durationSeconds) {
        List<InputEvent> events = new ArrayList<>();
        for (int port : ports) {
            Random random = new Random(seed * 0xBF58476D1CE4E5B9L + port * 0x94D049BB133111EBL + 17L);
            buttons(random, port, durationSeconds, events);
            axes(random, port, durationSeconds, events);
            povs(random, port, durationSeconds, events);
        }
        events.sort((a, b) -> Double.compare(a.timeSeconds(), b.timeSeconds()));
        return List.copyOf(events);
    }

    private void buttons(Random random, int port, double duration, List<InputEvent> out) {
        if (buttons == 0 || buttonPressesPerSecond == 0.0) return;
        double[] releasedAt = new double[buttons + 1];
        double time = exponential(random, buttonPressesPerSecond);
        while (time < duration) {
            int button = 1 + random.nextInt(buttons);
            if (time >= releasedAt[button]) {
                double release = time + Math.max(0.05, exponential(random, 1.0 / meanHoldSeconds));
                releasedAt[button] = release;
                out.add(new InputEvent(round(time), port, InputEvent.Kind.BUTTON, button, 1.0));
                if (release < duration) {
                    out.add(new InputEvent(round(release), port, InputEvent.Kind.BUTTON, button, 0.0));
                }
            }
            time += exponential(random, buttonPressesPerSecond);
        }
    }

    private void axes(Random random, int port, double duration, List<InputEvent> out) {
        if (axes == 0 || axisUpdatesPerSecond == 0.0) return;
        double period = 1.0 / axisUpdatesPerSecond;
        for (int axis = 0; axis < axes; axis++) {
            double value = 0.0;
            for (double time = period; time < duration; time += period) {
                double next = clamp(value + random.nextGaussian() * axisStep);
                double rounded = round3(next);
                if (rounded != round3(value)) {
                    out.add(new InputEvent(round(time), port, InputEvent.Kind.AXIS, axis, rounded));
                }
                value = next;
            }
        }
    }

    private void povs(Random random, int port, double duration, List<InputEvent> out) {
        if (povChangesPerSecond == 0.0) return;
        double time = exponential(random, povChangesPerSecond);
        while (time < duration) {
            int angle = random.nextInt(9) == 8 ? -1 : random.nextInt(8) * 45;
            out.add(new InputEvent(round(time), port, InputEvent.Kind.POV, 0, angle));
            time += exponential(random, povChangesPerSecond);
        }
    }

    private static double exponential(Random random, double rate) {
        return -Math.log(1.0 - random.nextDouble()) / rate;
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private static double round(double seconds) {
        return Math.round(seconds * 1000.0) / 1000.0;
    }

    private static double round3(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }

    private static double nonNegative(double value, String name) {
        if (!(value >= 0.0) || !Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite and non-negative");
        }
        return value;
    }

    private static double positive(double value, String name) {
        if (!(value > 0.0) || !Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
        return value;
    }
}
