package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.device.MotorCurrentLimits;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;

/** Configuration for detecting a commanded motor that is not moving. */
public record MotorStallSignal(
        MotorDevice motor,
        double currentLimitFraction,
        double velocityThresholdRotationsPerSecond,
        double commandedVoltageThreshold,
        double durationSeconds,
        double rearmSeconds) {
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

    public MotorStallSignal atCurrentLimit(double fraction) {
        return new MotorStallSignal(motor, fraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, rearmSeconds);
    }

    public MotorStallSignal velocityBelow(double rotationsPerSecond) {
        return new MotorStallSignal(motor, currentLimitFraction, rotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, rearmSeconds);
    }

    /** Sets the minimum open-loop command, expressed as equivalent volts. */
    public MotorStallSignal commandedVoltageAbove(double volts) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                volts, durationSeconds, rearmSeconds);
    }

    public MotorStallSignal forSeconds(double seconds) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, seconds, rearmSeconds);
    }

    public MotorStallSignal rearmAfterSeconds(double seconds) {
        return new MotorStallSignal(motor, currentLimitFraction, velocityThresholdRotationsPerSecond,
                commandedVoltageThreshold, durationSeconds, seconds);
    }

    public double currentThresholdAmps() {
        return motor.currentLimits().stallDetectionAmps() * currentLimitFraction;
    }

    public boolean instantaneousActive() {
        return commanded()
                && currentAmps() >= currentThresholdAmps()
                && Math.abs(velocityRotationsPerSecond()) <= velocityThresholdRotationsPerSecond;
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
