package ca.frc6390.athena.wpilib.controllers;

import ca.frc6390.athena.runtime.control.Debouncer;
import ca.frc6390.athena.runtime.control.ModifiedAxis;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Named driver-station controller profile for WPILib robot projects.
 */
public final class WpilibDriverStationProfile {
    /**
     * Default driver role name.
     */
    public static final String DRIVER = "driver";
    /**
     * Default operator role name.
     */
    public static final String OPERATOR = "operator";

    private final Map<String, Integer> ports = new LinkedHashMap<>();
    private double deadzone = 0.1;
    private double debounceSeconds = 0.2;

    private WpilibDriverStationProfile() {
    }

    /**
     * Creates a profile with driver and operator controller ports.
     *
     * @param driverPort driver controller port
     * @param operatorPort operator controller port
     * @return profile
     */
    public static WpilibDriverStationProfile standard(int driverPort, int operatorPort) {
        return new WpilibDriverStationProfile()
                .role(DRIVER, driverPort)
                .role(OPERATOR, operatorPort);
    }

    /**
     * Adds or replaces a named controller role.
     *
     * @param role role name
     * @param port driver-station port
     * @return this profile
     */
    public WpilibDriverStationProfile role(String role, int port) {
        ports.put(normalizeRole(role), Math.max(0, port));
        return this;
    }

    /**
     * Sets the default axis deadzone.
     *
     * @param deadzone axis deadzone
     * @return this profile
     */
    public WpilibDriverStationProfile deadzone(double deadzone) {
        this.deadzone = Double.isFinite(deadzone) && deadzone >= 0.0 ? Math.min(deadzone, 0.99) : 0.0;
        return this;
    }

    /**
     * Sets the default button debounce duration.
     *
     * @param debounceSeconds debounce duration
     * @return this profile
     */
    public WpilibDriverStationProfile debounceSeconds(double debounceSeconds) {
        this.debounceSeconds = Double.isFinite(debounceSeconds) && debounceSeconds >= 0.0 ? debounceSeconds : 0.0;
        return this;
    }

    /**
     * Returns a configured role port.
     *
     * @param role role name
     * @return driver-station port
     */
    public int port(String role) {
        Integer port = ports.get(normalizeRole(role));
        if (port == null) {
            throw new IllegalArgumentException("No driver-station controller role named " + role + ".");
        }
        return port;
    }

    /**
     * Creates an Xbox controller for a role.
     *
     * @param role role name
     * @return WPILib Xbox controller
     */
    public XboxController xbox(String role) {
        return new XboxController(port(role));
    }

    /**
     * Creates a joystick for a role.
     *
     * @param role role name
     * @return WPILib joystick
     */
    public Joystick joystick(String role) {
        return new Joystick(port(role));
    }

    /**
     * Creates a shaped axis using this profile's default deadzone.
     *
     * @param reader indexed axis reader
     * @param axis Xbox axis
     * @return shaped axis
     */
    public ModifiedAxis axis(WpilibControllerBindings.AxisReader reader, XboxController.Axis axis) {
        Objects.requireNonNull(axis, "axis");
        return WpilibControllerBindings.axis(reader, axis.value, deadzone);
    }

    /**
     * Creates a shaped driver axis using this profile's default deadzone.
     *
     * @param reader indexed axis reader
     * @param axis Xbox axis
     * @return shaped axis
     */
    public ModifiedAxis driverAxis(WpilibControllerBindings.AxisReader reader, XboxController.Axis axis) {
        return axis(reader, axis);
    }

    /**
     * Creates a shaped operator axis using this profile's default deadzone.
     *
     * @param reader indexed axis reader
     * @param axis Xbox axis
     * @return shaped axis
     */
    public ModifiedAxis operatorAxis(WpilibControllerBindings.AxisReader reader, XboxController.Axis axis) {
        return axis(reader, axis);
    }

    /**
     * Creates a debounced button using this profile's default debounce.
     *
     * @param reader indexed button reader
     * @param button Xbox button
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public Debouncer button(
            WpilibControllerBindings.ButtonReader reader,
            XboxController.Button button,
            DoubleSupplier clockSeconds) {
        Objects.requireNonNull(button, "button");
        return WpilibControllerBindings.button(reader, button.value, debounceSeconds, clockSeconds);
    }

    /**
     * Creates a debounced driver button using this profile's default debounce.
     *
     * @param reader indexed button reader
     * @param button Xbox button
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public Debouncer driverButton(
            WpilibControllerBindings.ButtonReader reader,
            XboxController.Button button,
            DoubleSupplier clockSeconds) {
        return button(reader, button, clockSeconds);
    }

    /**
     * Creates a debounced operator button using this profile's default debounce.
     *
     * @param reader indexed button reader
     * @param button Xbox button
     * @param clockSeconds clock in seconds
     * @return debounced button
     */
    public Debouncer operatorButton(
            WpilibControllerBindings.ButtonReader reader,
            XboxController.Button button,
            DoubleSupplier clockSeconds) {
        return button(reader, button, clockSeconds);
    }

    private static String normalizeRole(String role) {
        return role == null || role.isBlank() ? DRIVER : role.trim();
    }
}
