package ca.frc6390.athena.hardware.power;

import ca.frc6390.athena.api.hardware.MotorKind;
import java.util.Locale;

/**
 * Nominal electrical and thermal constants used by Athena's motor model.
 *
 * <p>Used for: the winding resistance {@code R} (heating and back-EMF based current estimate), the
 * speed constant {@code Kv} (back-EMF), the lumped thermal capacity {@code C} and cooling time
 * constant {@code tau}, and the default derate curve.</p>
 *
 * <p>Lumped model: {@code dT/dt = I^2 R / C - (T - T_ambient) / tau}.</p>
 *
 * <p>The values below are <b>nominal placeholders</b> derived from typical published stall current,
 * free speed and resistance figures. They are not calibrated. Replace them with characterized values
 * (for example through the characterization thermal run) before relying on temperature estimates.
 * Motor kinds not listed use {@link #GENERIC}.</p>
 *
 * @param resistanceOhms phase-to-phase winding resistance estimate
 * @param kvRadPerSecondPerVolt no-load speed constant in rad/s per volt
 * @param heatCapacityJoulesPerCelsius lumped thermal capacity
 * @param coolingSeconds first-order cooling time constant
 * @param ambientCelsius ambient temperature assumed at boot
 * @param maxWindingCelsius rated maximum winding temperature used by the default derate curve
 */
public record MotorThermalSpec(
        double resistanceOhms,
        double kvRadPerSecondPerVolt,
        double heatCapacityJoulesPerCelsius,
        double coolingSeconds,
        double ambientCelsius,
        double maxWindingCelsius) {

    /** Fallback constants for motor kinds without a table entry. */
    public static final MotorThermalSpec GENERIC = new MotorThermalSpec(
            0.05, 500.0 * 2.0 * Math.PI / 60.0, 200.0, 300.0, 25.0, 130.0);

    public MotorThermalSpec {
        if (!(resistanceOhms > 0.0) || !(kvRadPerSecondPerVolt > 0.0)
                || !(heatCapacityJoulesPerCelsius > 0.0) || !(coolingSeconds > 0.0)
                || !(maxWindingCelsius > ambientCelsius)) {
            throw new IllegalArgumentException("Motor thermal constants must be positive.");
        }
    }

    /** Returns the default derate curve: full limit until 30 C below the rated winding temperature, then to 50 percent. */
    public Thermal defaultDerate() {
        return new Thermal(maxWindingCelsius - 30.0, maxWindingCelsius, 0.5);
    }

    /**
     * Returns the constants for a motor kind.
     *
     * @param kind motor kind, may be {@code null}
     * @return constants, {@link #GENERIC} when the kind is not in the table
     */
    public static MotorThermalSpec forKind(MotorKind kind) {
        if (kind == null) return GENERIC;
        return switch (kind.motorKey().toLowerCase(Locale.ROOT)) {
            // Kv values are free speed / nominal voltage; resistance is nominal voltage / stall current.
            case "kraken-x60" -> new MotorThermalSpec(0.033, 500.0 * TWO_PI_OVER_60, 260.0, 300.0, 25.0, 150.0);
            case "kraken-x44" -> new MotorThermalSpec(0.055, 420.0 * TWO_PI_OVER_60, 170.0, 280.0, 25.0, 150.0);
            case "falcon-500" -> new MotorThermalSpec(0.047, 531.0 * TWO_PI_OVER_60, 260.0, 300.0, 25.0, 150.0);
            case "minion" -> new MotorThermalSpec(0.030, 500.0 * TWO_PI_OVER_60, 160.0, 280.0, 25.0, 150.0);
            case "neo" -> new MotorThermalSpec(0.114, 473.0 * TWO_PI_OVER_60, 180.0, 260.0, 25.0, 130.0);
            case "neo-550" -> new MotorThermalSpec(0.130, 917.0 * TWO_PI_OVER_60, 120.0, 240.0, 25.0, 130.0);
            case "neo-vortex" -> new MotorThermalSpec(0.057, 565.0 * TWO_PI_OVER_60, 200.0, 260.0, 25.0, 130.0);
            case "cim" -> new MotorThermalSpec(0.090, 444.0 * TWO_PI_OVER_60, 350.0, 320.0, 25.0, 130.0);
            case "mini-cim" -> new MotorThermalSpec(0.135, 487.0 * TWO_PI_OVER_60, 180.0, 280.0, 25.0, 130.0);
            case "vex-775-pro" -> new MotorThermalSpec(0.090, 1561.0 * TWO_PI_OVER_60, 300.0, 300.0, 25.0, 130.0);
            default -> GENERIC;
        };
    }

    private static final double TWO_PI_OVER_60 = 2.0 * Math.PI / 60.0;
}
