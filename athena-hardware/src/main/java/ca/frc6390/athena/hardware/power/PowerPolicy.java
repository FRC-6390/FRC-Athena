package ca.frc6390.athena.hardware.power;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

/**
 * Robot-wide electrical policy. Declare one typed field on the robot mechanism tree; Athena discovers it
 * like an {@code AutoChooser}. When absent, {@link #defaults()} applies.
 *
 * <pre>{@code
 * public final PowerPolicy power = PowerPolicy.defaults()
 *         .floor(Volts.of(7.5))
 *         .staggerStarts(Seconds.of(0.2));
 * }</pre>
 *
 * <ul>
 *   <li>{@code floor} – the battery voltage the robot tries to keep. When the predicted bus voltage
 *       falls below it, motor current is budgeted by share. Default 7.0 V.</li>
 *   <li>{@code staggerStarts} – minimum time between two large current rises (motor starts) on
 *       different motors. Default zero (disabled).</li>
 *   <li>{@code estimateBatteryResistance} – whether the internal resistance is estimated online. Default
 *       true. When false, {@link #defaultBatteryOhms()} is used.</li>
 * </ul>
 */
public final class PowerPolicy {
    /** Default bus-voltage floor in volts. */
    public static final double DEFAULT_FLOOR_VOLTS = 7.0;
    /** Default battery internal resistance in ohms, used before estimation has enough excitation. */
    public static final double DEFAULT_BATTERY_OHMS = 0.02;

    private final double floorVolts;
    private final double staggerSeconds;
    private final boolean estimateBatteryResistance;

    private PowerPolicy(double floorVolts, double staggerSeconds, boolean estimateBatteryResistance) {
        this.floorVolts = floorVolts;
        this.staggerSeconds = staggerSeconds;
        this.estimateBatteryResistance = estimateBatteryResistance;
    }

    /** Returns the default policy. */
    public static PowerPolicy defaults() {
        return new PowerPolicy(DEFAULT_FLOOR_VOLTS, 0.0, true);
    }

    /**
     * Sets the bus-voltage floor.
     *
     * @param floor positive finite voltage
     * @return updated policy
     */
    public PowerPolicy floor(Voltage floor) {
        double volts = Objects.requireNonNull(floor, "floor").in(Units.Volts);
        if (!Double.isFinite(volts) || volts <= 0.0) {
            throw new IllegalArgumentException("Power floor must be a positive finite voltage.");
        }
        return new PowerPolicy(volts, staggerSeconds, estimateBatteryResistance);
    }

    /**
     * Sets the minimum spacing between large current rises.
     *
     * @param spacing non-negative time
     * @return updated policy
     */
    public PowerPolicy staggerStarts(Time spacing) {
        double seconds = Objects.requireNonNull(spacing, "spacing").in(Units.Seconds);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            throw new IllegalArgumentException("Stagger spacing must be a non-negative finite time.");
        }
        return new PowerPolicy(floorVolts, seconds, estimateBatteryResistance);
    }

    /**
     * Enables or disables online battery resistance estimation.
     *
     * @param enabled true to estimate online
     * @return updated policy
     */
    public PowerPolicy estimateBatteryResistance(boolean enabled) {
        return new PowerPolicy(floorVolts, staggerSeconds, enabled);
    }

    /** Returns the bus-voltage floor in volts. */
    public double floorVolts() { return floorVolts; }

    /** Returns the stagger spacing in seconds (zero when disabled). */
    public double staggerSeconds() { return staggerSeconds; }

    /** Returns whether battery resistance is estimated online. */
    public boolean estimateBatteryResistance() { return estimateBatteryResistance; }

    /** Returns the battery resistance used before estimation is available. */
    public double defaultBatteryOhms() { return DEFAULT_BATTERY_OHMS; }
}
