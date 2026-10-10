package ca.frc6390.athena.robot.explore;

import java.util.Locale;
import java.util.Objects;

/**
 * One controller change in an exploration input script.
 *
 * <p>The script is a time-ordered list of these events. The controller state at any time is the result of
 * applying every event up to that time, so removing events (shrinking) is well defined.</p>
 *
 * @param timeSeconds time since the start of the run
 * @param port zero-based joystick port
 * @param kind what changed
 * @param index WPILib one-based button, axis index, or POV index depending on {@code kind}
 * @param value 1 or 0 for buttons, the axis value in [-1, 1], or the POV angle in degrees (-1 when released)
 */
public record InputEvent(double timeSeconds, int port, Kind kind, int index, double value) {
    /** Kind of controller change. */
    public enum Kind {
        /** Button {@code index} (one-based) pressed when {@code value} is 1, released when 0. */
        BUTTON,
        /** Axis {@code index} set to {@code value}. */
        AXIS,
        /** POV {@code index} set to {@code value} degrees, -1 when released. */
        POV
    }

    public InputEvent {
        Objects.requireNonNull(kind, "kind");
    }

    /** Returns a compact one-line description, for example {@code t=12.340 p0 button 4 down}. */
    public String describe() {
        String what = switch (kind) {
            case BUTTON -> "button " + index + (value != 0.0 ? " down" : " up");
            case AXIS -> "axis " + index + " = " + String.format(Locale.ROOT, "%.3f", value);
            case POV -> "pov " + index + " = " + (int) value;
        };
        return String.format(Locale.ROOT, "t=%.3f p%d %s", timeSeconds, port, what);
    }
}
