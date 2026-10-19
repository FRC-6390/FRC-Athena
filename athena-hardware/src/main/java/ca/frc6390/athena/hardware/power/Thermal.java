package ca.frc6390.athena.hardware.power;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Temperature;
import java.util.Objects;

/**
 * Thermal derate curve for a motor's current limits.
 *
 * <p>Below {@code start} the motor is at full limit. Between {@code start} and {@code end} the limit
 * falls linearly to {@code minFraction}, and stays there above {@code end}. The thermal load is the
 * position on that ramp, in {@code [0, 1]}. Thermal never brakes, holds or locks a motor out: it only
 * scales the current limit.</p>
 *
 * @param start temperature where derating begins
 * @param end temperature where the minimum fraction is reached
 * @param minFraction fraction of the limit kept at or above {@code end}, in {@code [0, 1]}
 */
public record Thermal(Temperature start, Temperature end, double minFraction) {
    public Thermal {
        Objects.requireNonNull(start, "start");
        Objects.requireNonNull(end, "end");
        double startCelsius = start.in(Units.Celsius);
        double endCelsius = end.in(Units.Celsius);
        if (!Double.isFinite(startCelsius) || !Double.isFinite(endCelsius) || endCelsius <= startCelsius) {
            throw new IllegalArgumentException("Thermal derate end must be above start.");
        }
        if (!(minFraction >= 0.0 && minFraction <= 1.0)) {
            throw new IllegalArgumentException("Thermal minimum fraction must be in [0, 1].");
        }
    }

    /**
     * Creates a derate curve.
     *
     * @param start temperature where derating begins
     * @param end temperature where the minimum is reached
     * @param minFraction limit fraction kept at high temperature, in {@code [0, 1]}
     * @return curve
     */
    public static Thermal derate(Temperature start, Temperature end, double minFraction) {
        return new Thermal(start, end, minFraction);
    }

    /**
     * Returns the fraction of the current limit allowed at a temperature.
     *
     * @param temperature motor temperature
     * @return fraction in {@code [minFraction, 1]}; 1 when the temperature is unknown (NaN)
     */
    public double derateFraction(Temperature temperature) {
        return derateFractionCelsius(temperature.in(Units.Celsius));
    }

    /**
     * Returns the thermal load: 0 at or below {@code start}, 1 at or above {@code end}.
     *
     * @param temperature motor temperature
     * @return load in {@code [0, 1]}; 0 when unknown
     */
    public double load(Temperature temperature) {
        return loadCelsius(temperature.in(Units.Celsius));
    }

    /** Per-cycle primitive: {@link #derateFraction(Temperature)} in Celsius. */
    public double derateFractionCelsius(double celsius) {
        if (Double.isNaN(celsius)) return 1.0;
        return 1.0 - (1.0 - minFraction) * loadCelsius(celsius);
    }

    /** Per-cycle primitive: {@link #load(Temperature)} in Celsius. */
    public double loadCelsius(double celsius) {
        if (Double.isNaN(celsius)) return 0.0;
        double startCelsius = start.in(Units.Celsius);
        double endCelsius = end.in(Units.Celsius);
        return Math.max(0.0, Math.min(1.0, (celsius - startCelsius) / (endCelsius - startCelsius)));
    }
}
