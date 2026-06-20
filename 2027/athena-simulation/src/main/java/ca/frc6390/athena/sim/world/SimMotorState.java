package ca.frc6390.athena.sim.world;

/**
 * Mutable motor simulation state.
 */
public final class SimMotorState {
    private final String name;
    private double percentOutput;
    private double velocityPerSecond;
    private double position;

    /**
     * Creates a motor simulation state.
     *
     * @param name motor name
     */
    public SimMotorState(String name) {
        this.name = name == null || name.isBlank() ? "motor" : name;
    }

    /**
     * Returns motor name.
     *
     * @return motor name
     */
    public String name() {
        return name;
    }

    /**
     * Returns last percent output.
     *
     * @return percent output
     */
    public double percentOutput() {
        return percentOutput;
    }

    /**
     * Sets percent output and clamps it to [-1, 1].
     *
     * @param percentOutput requested percent output
     * @return this state
     */
    public SimMotorState percentOutput(double percentOutput) {
        if (!Double.isFinite(percentOutput)) {
            this.percentOutput = 0.0;
        } else {
            this.percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        }
        return this;
    }

    /**
     * Returns simulated velocity in mechanism units per second.
     *
     * @return velocity
     */
    public double velocityPerSecond() {
        return velocityPerSecond;
    }

    /**
     * Sets simulated velocity.
     *
     * @param velocityPerSecond velocity in mechanism units per second
     * @return this state
     */
    public SimMotorState velocityPerSecond(double velocityPerSecond) {
        this.velocityPerSecond = Double.isFinite(velocityPerSecond) ? velocityPerSecond : 0.0;
        return this;
    }

    /**
     * Returns simulated position in mechanism units.
     *
     * @return position
     */
    public double position() {
        return position;
    }

    /**
     * Sets simulated position.
     *
     * @param position position in mechanism units
     * @return this state
     */
    public SimMotorState position(double position) {
        this.position = Double.isFinite(position) ? position : 0.0;
        return this;
    }

    /**
     * Advances position from velocity.
     *
     * @param seconds timestep in seconds
     * @return this state
     */
    public SimMotorState step(double seconds) {
        if (Double.isFinite(seconds) && seconds > 0.0) {
            position += velocityPerSecond * seconds;
        }
        return this;
    }
}
