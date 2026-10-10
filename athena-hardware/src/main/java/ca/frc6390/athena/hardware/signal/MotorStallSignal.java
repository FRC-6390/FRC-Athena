package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorCurrentLimits;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;

/**
 * Configuration for detecting a commanded motor that is not moving.
 *
 * <p>Two detectors share this type. The <em>model-based</em> detector is the default when the motor
 * model has built-in {@link MotorConstants}. It has no fixed voltage threshold. A stall is reported
 * when the motor draws at least twice its free-run current, and its measured speed is below 20% of
 * the no-load speed that the applied voltage would produce, for at least 0.1 s.</p>
 *
 * <p>The <em>legacy</em> detector uses fixed thresholds. It is selected by
 * {@link #atCurrentLimit(double)}, {@link #velocityBelow(double)} or
 * {@link #commandedVoltageAbove(double)}, and it is used when the motor model has no constants.</p>
 */
public record MotorStallSignal(
        MotorDevice motor,
        double currentLimitFraction,
        double velocityThresholdRotationsPerSecond,
        double commandedVoltageThreshold,
        double durationSeconds,
        double rearmSeconds,
        MotorConstants constants) {

    /** Debounce time of the model-based detector. */
    public static final double MODEL_DURATION_SECONDS = 0.1;
    /** Measured speed below this fraction of the expected no-load speed counts as stalled. */
    public static final double MODEL_STALLED_SPEED_FRACTION = 0.2;
    /** Expected speed below this fraction of free speed is too small to judge a stall. */
    public static final double MODEL_MIN_EXPECTED_FRACTION = 0.1;
    /** Current must exceed this multiple of the free-run current to count as loaded. */
    public static final double MODEL_LOADED_CURRENT_MULTIPLE = 2.0;

    public MotorStallSignal {
        Objects.requireNonNull(motor, "motor");
        if (!(currentLimitFraction > 0.0) || !Double.isFinite(currentLimitFraction)) {
            throw new IllegalArgumentException("Current-limit fraction must be positive and finite.");
        }
        if (velocityThresholdRotationsPerSecond < 0.0 || commandedVoltageThreshold < 0.0
                || durationSeconds < 0.0 || rearmSeconds < 0.0) {
            throw new IllegalArgumentException("Stall thresholds and timing cannot be negative.");
        }
        MotorCurrentLimits limits = motor.currentLimits();
        if (!limits.supportsStallDetection()) {
            throw new IllegalStateException("Motor " + motor.defaultName()
                    + " needs a current limit before stall detection can be used.");
        }
    }

    /** Creates the legacy fixed-threshold detector. */
    public MotorStallSignal(
            MotorDevice motor,
            double currentLimitFraction,
            double velocityThresholdRotationsPerSecond,
            double commandedVoltageThreshold,
            double durationSeconds,
            double rearmSeconds) {
        this(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, rearmSeconds, null);
    }

    /** Creates the model-based detector for a motor with known constants. */
    public static MotorStallSignal modelBased(MotorDevice motor, MotorConstants constants) {
        return new MotorStallSignal(motor, 0.9, 1.0, 2.0, MODEL_DURATION_SECONDS, 0.25,
                Objects.requireNonNull(constants, "constants"));
    }

    /** Returns whether this detector is the model-based one (no fixed thresholds). */
    public boolean modelBased() {
        return constants != null;
    }

    /** Switches to the legacy detector with a current-limit fraction. */
    public MotorStallSignal atCurrentLimit(double fraction) {
        return new MotorStallSignal(motor, fraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, rearmSeconds, null);
    }

    /** Switches to the legacy detector with a velocity threshold. */
    public MotorStallSignal velocityBelow(double rotationsPerSecond) {
        return new MotorStallSignal(motor, currentLimitFraction, rotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, rearmSeconds, null);
    }

    /** Sets the minimum open-loop command, expressed as equivalent volts. Switches to the legacy detector. */
    public MotorStallSignal commandedVoltageAbove(double volts) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                volts, durationSeconds, rearmSeconds, null);
    }

    public MotorStallSignal forSeconds(double seconds) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, seconds, rearmSeconds, constants);
    }

    public MotorStallSignal rearmAfterSeconds(double seconds) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, seconds, constants);
    }

    public double currentThresholdAmps() {
        return motor.currentLimits().stallDetectionAmps() * currentLimitFraction;
    }

    public boolean instantaneousActive() {
        if (constants != null) {
            return modelStalled();
        }
        return commanded()
                && currentAmps() >= currentThresholdAmps()
                && Math.abs(velocityRotationsPerSecond()) <= velocityThresholdRotationsPerSecond;
    }

    private boolean modelStalled() {
        double volts;
        double amps;
        try {
            volts = motor.appliedVoltage();
            amps = currentAmps();
        } catch (UnsupportedOperationException unavailable) {
            return false;
        }
        if (!Double.isFinite(volts) || !Double.isFinite(amps)) {
            return false;
        }
        double freeSpeedRps = constants.freeSpeedRadPerSec() / (2.0 * Math.PI);
        double expectedRps = constants.expectedUnloadedSpeedRadPerSec(volts) / (2.0 * Math.PI);
        if (expectedRps < MODEL_MIN_EXPECTED_FRACTION * freeSpeedRps) {
            return false;
        }
        if (amps < MODEL_LOADED_CURRENT_MULTIPLE * constants.freeCurrentAmps()) {
            return false;
        }
        return Math.abs(velocityRotationsPerSecond()) < MODEL_STALLED_SPEED_FRACTION * expectedRps;
    }

    /** Returns whether Athena's latest command requests movement. */
    public boolean commanded() {
        return motor.command().requestsMovement(commandedVoltageThreshold, velocityThresholdRotationsPerSecond);
    }

    /** Returns the current channel selected from the configured current limits. */
    public double currentAmps() {
        MotorCurrentLimits limits = motor.currentLimits();
        return limits.statorAmps() > 0 ? motor.statorCurrentAmps() : motor.supplyCurrentAmps();
    }

    /** Returns the motor's measured velocity used by this detector. */
    public double velocityRotationsPerSecond() {
        return motor.velocityRotationsPerSecond();
    }
}
