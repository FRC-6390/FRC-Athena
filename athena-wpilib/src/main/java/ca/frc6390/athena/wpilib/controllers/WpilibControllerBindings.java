package ca.frc6390.athena.wpilib.controllers;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.runtime.control.Debouncer;
import ca.frc6390.athena.runtime.control.ModifiedAxis;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Controller binding helpers for WPILib-shaped input suppliers.
 */
public final class WpilibControllerBindings {
    private WpilibControllerBindings() {
    }

    /**
     * Wraps a joystick axis with Athena axis shaping.
     *
     * @param rawAxis raw axis supplier
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis axis(DoubleSupplier rawAxis, double deadzone) {
        return new ModifiedAxis(Objects.requireNonNull(rawAxis, "rawAxis"), deadzone);
    }

    /**
     * Wraps a raw WPILib HID axis with Athena axis shaping.
     *
     * @param hid WPILib HID device
     * @param axis raw axis index
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis axis(GenericHID hid, int axis, double deadzone) {
        Objects.requireNonNull(hid, "hid");
        return axis(hid::getRawAxis, axis, deadzone);
    }

    /**
     * Wraps an indexed axis reader with Athena axis shaping.
     *
     * @param reader indexed axis reader
     * @param axis raw axis index
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis axis(AxisReader reader, int axis, double deadzone) {
        Objects.requireNonNull(reader, "reader");
        return axis(() -> reader.readAxis(axis), deadzone);
    }

    /**
     * Wraps an Xbox controller axis with Athena axis shaping.
     *
     * @param controller WPILib Xbox controller
     * @param axis Xbox axis
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis axis(XboxController controller, XboxController.Axis axis, double deadzone) {
        Objects.requireNonNull(axis, "axis");
        return axis((GenericHID) controller, axis.value, deadzone);
    }

    /**
     * Wraps a joystick X axis with Athena axis shaping.
     *
     * @param joystick WPILib joystick
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis joystickX(Joystick joystick, double deadzone) {
        Objects.requireNonNull(joystick, "joystick");
        return axis(joystick::getX, deadzone);
    }

    /**
     * Wraps a joystick Y axis with Athena axis shaping.
     *
     * @param joystick WPILib joystick
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis joystickY(Joystick joystick, double deadzone) {
        Objects.requireNonNull(joystick, "joystick");
        return axis(joystick::getY, deadzone);
    }

    /**
     * Wraps a joystick twist axis with Athena axis shaping.
     *
     * @param joystick WPILib joystick
     * @param deadzone deadzone
     * @return shaped axis
     */
    public static ModifiedAxis joystickTwist(Joystick joystick, double deadzone) {
        Objects.requireNonNull(joystick, "joystick");
        return axis(joystick::getTwist, deadzone);
    }

    /**
     * Wraps a button supplier with deterministic debounce behavior.
     *
     * @param rawButton raw button supplier
     * @param debounceSeconds debounce duration
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public static Debouncer button(BooleanSupplier rawButton, double debounceSeconds, DoubleSupplier clockSeconds) {
        return new Debouncer(Objects.requireNonNull(rawButton, "rawButton"), debounceSeconds, clockSeconds);
    }

    /**
     * Wraps a raw WPILib HID button with deterministic debounce behavior.
     *
     * @param hid WPILib HID device
     * @param button raw button index
     * @param debounceSeconds debounce duration
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public static Debouncer button(
            GenericHID hid, int button, double debounceSeconds, DoubleSupplier clockSeconds) {
        Objects.requireNonNull(hid, "hid");
        return button(hid::getRawButton, button, debounceSeconds, clockSeconds);
    }

    /**
     * Wraps an indexed button reader with deterministic debounce behavior.
     *
     * @param reader indexed button reader
     * @param button raw button index
     * @param debounceSeconds debounce duration
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public static Debouncer button(
            ButtonReader reader, int button, double debounceSeconds, DoubleSupplier clockSeconds) {
        Objects.requireNonNull(reader, "reader");
        return button(() -> reader.readButton(button), debounceSeconds, clockSeconds);
    }

    /**
     * Wraps an Xbox controller button with deterministic debounce behavior.
     *
     * @param controller WPILib Xbox controller
     * @param button Xbox button
     * @param debounceSeconds debounce duration
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public static Debouncer button(
            XboxController controller,
            XboxController.Button button,
            double debounceSeconds,
            DoubleSupplier clockSeconds) {
        Objects.requireNonNull(button, "button");
        return button((GenericHID) controller, button.value, debounceSeconds, clockSeconds);
    }

    /**
     * Wraps a joystick trigger with deterministic debounce behavior.
     *
     * @param joystick WPILib joystick
     * @param debounceSeconds debounce duration
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public static Debouncer joystickTrigger(
            Joystick joystick, double debounceSeconds, DoubleSupplier clockSeconds) {
        Objects.requireNonNull(joystick, "joystick");
        return button(joystick::getTrigger, debounceSeconds, clockSeconds);
    }

    /**
     * Indexed controller axis reader.
     */
    @FunctionalInterface
    public interface AxisReader {
        /**
         * Reads a raw axis value.
         *
         * @param axis raw axis index
         * @return axis value
         */
        double readAxis(int axis);
    }

    /**
     * Indexed controller button reader.
     */
    @FunctionalInterface
    public interface ButtonReader {
        /**
         * Reads a raw button value.
         *
         * @param button raw button index
         * @return button value
         */
        boolean readButton(int button);
    }
}
