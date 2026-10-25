package ca.frc6390.athena.hardware.power;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

/**
 * Bus-sag response for a {@link PowerProfile}: while the filtered battery voltage is below the
 * threshold, the motor's current cap is reduced by up to {@code shedFraction} (0.5 halves it). The
 * reduction grows linearly from none at the threshold to the full fraction
 * {@link PowerPlanner#SAG_BAND_VOLTS} below it, and the cap moves at bounded rates, so the response does not
 * switch on and off against the battery.
 *
 * @param shedFraction fraction of the cap removed at full sag, in {@code (0, 1]}
 * @param threshold battery voltage below which the response applies
 */
public record Sag(double shedFraction, Voltage threshold) {
    public Sag {
        if (!(shedFraction > 0.0 && shedFraction <= 1.0)) {
            throw new IllegalArgumentException("Sag shed fraction must be in (0, 1].");
        }
        Objects.requireNonNull(threshold, "threshold");
        if (!Double.isFinite(threshold.in(Units.Volts)) || threshold.in(Units.Volts) <= 0.0) {
            throw new IllegalArgumentException("Sag threshold must be a positive finite voltage.");
        }
    }

    /**
     * Starts a sag response that removes {@code fraction} of the cap.
     *
     * @param fraction fraction of the cap to remove, in {@code (0, 1]}
     * @return builder completed with {@link Shed#below(Voltage)}
     */
    public static Shed shed(double fraction) {
        return new Shed(fraction);
    }

    /** First half of a sag declaration. */
    public record Shed(double fraction) {
        public Shed {
            if (!(fraction > 0.0 && fraction <= 1.0)) {
                throw new IllegalArgumentException("Sag shed fraction must be in (0, 1].");
            }
        }

        /** Completes the declaration with the bus voltage below which the response applies. */
        public Sag below(Voltage voltage) {
            Objects.requireNonNull(voltage, "voltage");
            return new Sag(fraction, voltage);
        }
    }
}
