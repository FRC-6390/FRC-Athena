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
 * @param startCelsius temperature where derating begins
 * @param endCelsius temperature where the minimum fraction is reached
 * @param minFraction fraction of the limit kept at or above {@code endCelsius}, in {@code [0, 1]}
 */
public record Thermal(double startCelsius, double endCelsius, double minFraction) {
    public Thermal {
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
        Objects.requireNonNull(start, "start");
        Objects.requireNonNull(end, "end");
        return new Thermal(start.in(Units.Celsius), end.in(Units.Celsius), minFraction);
    }

    /**
     * Returns the fraction of the current limit allowed at a temperature.
     *
     * @param celsius motor temperature
     * @return fraction in {@code [minFraction, 1]}; 1 when the temperature is unknown (NaN)
     */
    public double derateFraction(double celsius) {
        if (Double.isNaN(celsius)) return 1.0;
        return 1.0 - (1.0 - minFraction) * load(celsius);
    }

    /**
     * Returns the thermal load: 0 at or below {@code start}, 1 at or above {@code end}.
     *
     * @param celsius motor temperature
     * @return load in {@code [0, 1]}; 0 when unknown
     */
    public double load(double celsius) {
        if (Double.isNaN(celsius)) return 0.0;
        return Math.max(0.0, Math.min(1.0, (celsius - startCelsius) / (endCelsius - startCelsius)));
    }
}
