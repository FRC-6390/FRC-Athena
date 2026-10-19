package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorCurrentLimits;
import ca.frc6390.athena.hardware.device.MotorDevice;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.Objects;

/**
 * Configuration for detecting a commanded motor that is not moving.
 *
 * <p>Two detectors share this type. The <em>model-based</em> detector is the default when the motor
 * model has built-in {@link MotorConstants}. It has no fixed voltage threshold. A stall is reported
 * when the motor draws at least twice its free-run current, and its measured speed is below 20% of
 * the no-load speed that the applied voltage would produce, for at least 0.1 s.</p>
 *
 * <p>The fixed-threshold detector uses fixed thresholds. It is selected by
 * {@link #atCurrentLimit(double)}, {@link #velocityBelow(AngularVelocity)} or
 * {@link #commandedVoltageAbove(Voltage)}, and it is used when the motor model has no constants.</p>
 *
 * <p>The current-limit threshold is derived once when the signal is declared, because a
 * {@link MotorDevice} is immutable. Per-cycle evaluation reads primitive values from the device and
 * does not allocate measures.</p>
 */
public final class MotorStallSignal {
    /** Debounce time of the model-based detector. */
    public static final double MODEL_DURATION_SECONDS = 0.1;
    /** Measured speed below this fraction of the expected no-load speed counts as stalled. */
    public static final double MODEL_STALLED_SPEED_FRACTION = 0.2;
    /** Expected speed below this fraction of free speed is too small to judge a stall. */
    public static final double MODEL_MIN_EXPECTED_FRACTION = 0.1;
    /** Current must exceed this multiple of the free-run current to count as loaded. */
    public static final double MODEL_LOADED_CURRENT_MULTIPLE = 2.0;

    private final MotorDevice motor;
    private final double currentLimitFraction;
    private final double velocityThresholdRotationsPerSecond;
    private final double commandedVoltageThresholdVolts;
    private final double durationSeconds;
    private final double rearmSeconds;
    private final MotorConstants constants;
    private final boolean statorChannel;
    private final double stallDetectionAmps;

    private MotorStallSignal(
            MotorDevice motor,
            double currentLimitFraction,
            double velocityThresholdRotationsPerSecond,
            double commandedVoltageThresholdVolts,
            double durationSeconds,
            double rearmSeconds,
            MotorConstants constants) {
        this.motor = Objects.requireNonNull(motor, "motor");
        if (!(currentLimitFraction > 0.0) || !Double.isFinite(currentLimitFraction)) {
            throw new IllegalArgumentException("Current-limit fraction must be positive and finite.");
        }
        if (velocityThresholdRotationsPerSecond < 0.0 || commandedVoltageThresholdVolts < 0.0
                || durationSeconds < 0.0 || rearmSeconds < 0.0) {
            throw new IllegalArgumentException("Stall thresholds and timing cannot be negative.");
        }
        MotorCurrentLimits limits = motor.currentLimits();
        if (!limits.supportsStallDetection()) {
            throw new IllegalStateException("Motor " + motor.defaultName()
                    + " needs a current limit before stall detection can be used.");
        }
        this.currentLimitFraction = currentLimitFraction;
        this.velocityThresholdRotationsPerSecond = velocityThresholdRotationsPerSecond;
        this.commandedVoltageThresholdVolts = commandedVoltageThresholdVolts;
        this.durationSeconds = durationSeconds;
        this.rearmSeconds = rearmSeconds;
        this.constants = constants;
        this.statorChannel = limits.stator().in(Units.Amps) > 0.0;
        this.stallDetectionAmps = limits.stallDetection().in(Units.Amps);
    }

    /** Creates the fixed-threshold detector. */
    public MotorStallSignal(
            MotorDevice motor,
            double currentLimitFraction,
            AngularVelocity velocityThreshold,
            Voltage commandedVoltageThreshold,
            Time duration,
            Time rearmAfter) {
        this(motor, currentLimitFraction,
                Objects.requireNonNull(velocityThreshold, "velocityThreshold").in(Units.RotationsPerSecond),
                Objects.requireNonNull(commandedVoltageThreshold, "commandedVoltageThreshold").in(Units.Volts),
                Objects.requireNonNull(duration, "duration").in(Units.Seconds),
                Objects.requireNonNull(rearmAfter, "rearmAfter").in(Units.Seconds),
                null);
    }

    /** Creates the model-based detector for a motor with known constants. */
    public static MotorStallSignal modelBased(MotorDevice motor, MotorConstants constants) {
        return new MotorStallSignal(motor, 0.9, 1.0, 2.0, MODEL_DURATION_SECONDS, 0.25,
                Objects.requireNonNull(constants, "constants"));
    }

    /** Returns the motor this detector watches. */
    public MotorDevice motor() {
        return motor;
    }

    /** Returns the fraction of the current limit that counts as stalled (fixed-threshold detector). */
    public double currentLimitFraction() {
        return currentLimitFraction;
    }

    /** Returns the velocity below which the fixed-threshold detector reports a stall. */
    public AngularVelocity velocityThreshold() {
        return Units.RotationsPerSecond.of(velocityThresholdRotationsPerSecond);
    }

    /** Returns the commanded voltage that must be exceeded for the fixed-threshold detector. */
    public Voltage commandedVoltageThreshold() {
        return Units.Volts.of(commandedVoltageThresholdVolts);
    }

    /** Returns how long the stall condition must hold before it is reported. */
    public Time duration() {
        return Units.Seconds.of(durationSeconds);
    }

    /** Returns how long after a stall the detector waits before it can report again. */
    public Time rearmAfter() {
        return Units.Seconds.of(rearmSeconds);
    }

    /** Returns the motor constants used by the model-based detector, or {@code null}. */
    public MotorConstants constants() {
        return constants;
    }

    /** Returns whether this detector is the model-based one (no fixed thresholds). */
    public boolean modelBased() {
        return constants != null;
    }

    /** Switches to the fixed-threshold detector with a current-limit fraction. */
    public MotorStallSignal atCurrentLimit(double fraction) {
        return new MotorStallSignal(motor, fraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThresholdVolts, durationSeconds, rearmSeconds, null);
    }

    /** Switches to the fixed-threshold detector with a velocity threshold. */
    public MotorStallSignal velocityBelow(AngularVelocity velocity) {
        return new MotorStallSignal(motor, currentLimitFraction,
                Objects.requireNonNull(velocity, "velocity").in(Units.RotationsPerSecond),
                commandedVoltageThresholdVolts, durationSeconds, rearmSeconds, null);
    }

    /** Sets the minimum open-loop command. Switches to the fixed-threshold detector. */
    public MotorStallSignal commandedVoltageAbove(Voltage voltage) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                Objects.requireNonNull(voltage, "voltage").in(Units.Volts), durationSeconds, rearmSeconds, null);
    }

    /** Sets how long the stall condition must hold before it is reported. */
    public MotorStallSignal forTime(Time duration) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThresholdVolts, Objects.requireNonNull(duration, "duration").in(Units.Seconds),
                rearmSeconds, constants);
    }

    /** Sets how long after a stall the detector waits before it can report again. */
    public MotorStallSignal rearmAfter(Time rearm) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThresholdVolts, durationSeconds,
                Objects.requireNonNull(rearm, "rearm").in(Units.Seconds), constants);
    }

    /** Per-cycle primitive: debounce time in seconds. Teams should use {@link #duration()}. */
    public double durationSeconds() {
        return durationSeconds;
    }

    /** Per-cycle primitive: rearm time in seconds. Teams should use {@link #rearmAfter()}. */
    public double rearmSeconds() {
        return rearmSeconds;
    }

    /** Returns the current threshold of the fixed-threshold detector. */
    public Current currentThreshold() {
        return Units.Amps.of(currentThresholdAmps());
    }

    /** Returns the current channel selected from the configured current limits. */
    public Current current() {
        return Units.Amps.of(currentAmps());
    }

    /** Returns the motor's measured velocity used by this detector. */
    public AngularVelocity velocity() {
        return Units.RotationsPerSecond.of(motor.velocityRotationsPerSecond());
    }

    /**
     * Per-cycle primitive: current threshold in amps. Athena's hot paths use this to avoid allocating
     * measures; teams should use {@link #currentThreshold()}.
     *
     * @return current threshold in amps
     */
    public double currentThresholdAmps() {
        return stallDetectionAmps * currentLimitFraction;
    }

    /**
     * Per-cycle primitive: current in amps from the configured channel.
     *
     * @return current in amps
     */
    public double currentAmps() {
        return statorChannel ? motor.statorCurrentAmps() : motor.supplyCurrentAmps();
    }

    /** Evaluates the detector against the current motor state, without debounce. */
    public boolean instantaneousActive() {
        if (constants != null) {
            return modelStalled();
        }
        return commanded()
                && currentAmps() >= currentThresholdAmps()
                && Math.abs(motor.velocityRotationsPerSecond()) <= velocityThresholdRotationsPerSecond;
    }

    private boolean modelStalled() {
        double volts;
        double amps;
        try {
            volts = motor.appliedVoltageVolts();
            amps = currentAmps();
        } catch (UnsupportedOperationException unavailable) {
            return false;
        }
        if (!Double.isFinite(volts) || !Double.isFinite(amps)) {
            return false;
        }
        double freeSpeedRps = constants.freeSpeed().in(Units.RotationsPerSecond);
        // The motor model is symmetric: driving in reverse (homing usually does) expects the same
        // speed magnitude, so compare magnitudes rather than signed values.
        double expectedRps = constants.expectedUnloadedSpeedRadPerSec(Math.abs(volts)) / (2.0 * Math.PI);
        if (expectedRps < MODEL_MIN_EXPECTED_FRACTION * freeSpeedRps) {
            return false;
        }
        if (Math.abs(amps) < MODEL_LOADED_CURRENT_MULTIPLE * constants.freeCurrent().in(Units.Amps)) {
            return false;
        }
        return Math.abs(motor.velocityRotationsPerSecond()) < MODEL_STALLED_SPEED_FRACTION * expectedRps;
    }

    /** Returns whether Athena's latest command requests movement. */
    public boolean commanded() {
        return motor.command().requestsMovement(commandedVoltageThresholdVolts, velocityThresholdRotationsPerSecond);
    }
}
