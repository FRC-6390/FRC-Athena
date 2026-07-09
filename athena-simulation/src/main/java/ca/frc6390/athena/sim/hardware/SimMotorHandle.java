package ca.frc6390.athena.sim.hardware;

import java.util.Objects;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * In-memory simulation motor handle.
 */
public final class SimMotorHandle implements MotorHandle {
    /**
     * Last command mode applied to this motor.
     */
    public enum CommandKind {
        NEUTRAL,
        PERCENT,
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    private final MotorDevice device;
    private CommandKind commandKind = CommandKind.NEUTRAL;
    private double commandValue;
    private double percentOutput;
    private double positionRotations;
    private double velocityRotationsPerSecond;

    /**
     * Creates a simulation motor handle.
     *
     * @param device motor declaration
     */
    public SimMotorHandle(MotorDevice device) {
        this.device = Objects.requireNonNull(device, "device");
    }

    @Override
    public MotorDevice device() {
        return device;
    }

    @Override
    public void stop() {
        commandKind = CommandKind.NEUTRAL;
        commandValue = 0.0;
        percentOutput = 0.0;
        velocityRotationsPerSecond = 0.0;
    }

    @Override
    public void setPercentOutput(double percent) {
        percentOutput = clamp(percent, -1.0, 1.0);
        commandKind = CommandKind.PERCENT;
        commandValue = percentOutput;
        velocityRotationsPerSecond = percentOutput;
    }

    @Override
    public void setVoltage(double volts) {
        double safeVolts = finiteOrZero(volts);
        percentOutput = clamp(safeVolts / 12.0, -1.0, 1.0);
        commandKind = CommandKind.VOLTAGE;
        commandValue = safeVolts;
        velocityRotationsPerSecond = percentOutput;
    }

    @Override
    public void setPositionTargetRotations(double rotations) {
        double safeRotations = finiteOrZero(rotations);
        commandKind = CommandKind.POSITION;
        commandValue = safeRotations;
        positionRotations = safeRotations;
        velocityRotationsPerSecond = 0.0;
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        double safeVelocity = finiteOrZero(rotationsPerSecond);
        commandKind = CommandKind.VELOCITY;
        commandValue = safeVelocity;
        velocityRotationsPerSecond = safeVelocity;
    }

    @Override
    public double integratedPositionRotations() {
        return positionRotations;
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        return velocityRotationsPerSecond;
    }

    /**
     * Returns the last command kind applied to this handle.
     *
     * @return command kind
     */
    public CommandKind commandKind() {
        return commandKind;
    }

    /**
     * Returns the last command value applied to this handle.
     *
     * @return command value
     */
    public double commandValue() {
        return commandValue;
    }

    /**
     * Sets simulated sensor state without changing the active command.
     *
     * @param positionRotations position in rotations
     * @param velocityRotationsPerSecond velocity in rotations per second
     * @return this handle
     */
    public SimMotorHandle state(double positionRotations, double velocityRotationsPerSecond) {
        this.positionRotations = finiteOrZero(positionRotations);
        this.velocityRotationsPerSecond = finiteOrZero(velocityRotationsPerSecond);
        return this;
    }

    /**
     * Advances integrated position from velocity.
     *
     * @param seconds timestep in seconds
     */
    public void step(double seconds) {
        if (Double.isFinite(seconds) && seconds > 0.0) {
            positionRotations += velocityRotationsPerSecond * seconds;
        }
    }

    private static double clamp(double value, double min, double max) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(min, Math.min(max, value));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
