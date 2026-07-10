package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
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
    private MotorClosedLoopRequest closedLoopRequest;
    private SimMotorHandle leader;
    private boolean followerInverted;

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
    public void follow(MotorHandle leader, boolean inverted) {
        if (!(leader instanceof SimMotorHandle simLeader)) {
            throw new IllegalArgumentException("Simulated motor can only follow another simulated motor.");
        }
        this.leader = simLeader;
        followerInverted = inverted;
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
        closedLoopRequest = null;
    }

    @Override
    public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        setPositionTargetRotations(rotations);
        closedLoopRequest = request;
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        double safeVelocity = finiteOrZero(rotationsPerSecond);
        commandKind = CommandKind.VELOCITY;
        commandValue = safeVelocity;
        velocityRotationsPerSecond = safeVelocity;
        closedLoopRequest = null;
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond, MotorClosedLoopRequest request) {
        setVelocityTargetRotationsPerSecond(rotationsPerSecond);
        closedLoopRequest = request;
    }

    @Override
    public MotorControlCapabilities controlCapabilities() {
        return new MotorControlCapabilities(true, true, true, true, true, true, 4);
    }

    @Override
    public double integratedPositionRotations() {
        return positionRotations;
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        return velocityRotationsPerSecond;
    }

    @Override
    public void setIntegratedPositionRotations(double rotations) {
        positionRotations = finiteOrZero(rotations);
    }

    @Override
    public boolean supportsIntegratedPositionSetting() {
        return true;
    }

    /**
     * Returns the last command kind applied to this handle.
     *
     * @return command kind
     */
    public CommandKind commandKind() {
        return leader == null ? commandKind : leader.commandKind();
    }

    /**
     * Returns the last command value applied to this handle.
     *
     * @return command value
     */
    public double commandValue() {
        return leader == null ? commandValue : direction() * leader.commandValue();
    }

    /**
     * Returns the last offloaded closed-loop request.
     *
     * @return closed-loop request, or null when the last command was not offloaded
     */
    public MotorClosedLoopRequest closedLoopRequest() {
        return leader == null ? closedLoopRequest : leader.closedLoopRequest();
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

    private double direction() {
        return followerInverted ? -1.0 : 1.0;
    }
}
