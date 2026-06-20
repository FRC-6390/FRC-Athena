package ca.frc6390.athena.hardware.sensor;

import ca.frc6390.athena.hardware.input.InputConfig;

/**
 * Student-facing sensor wrapper declaration.
 */
public final class SensorConfig {
    private SensorKind kind = SensorKind.BUTTON;
    private InputConfig input;
    private boolean inverted = true;
    private boolean hardstop;
    private BlockDirection blockDirection = BlockDirection.NONE;
    private double position;

    private SensorConfig() {
    }

    /**
     * Creates a sensor config.
     *
     * @return sensor config
     */
    public static SensorConfig create() {
        return new SensorConfig();
    }

    /**
     * Declares a limit switch.
     *
     * @param channel digital input channel
     * @return this config
     */
    public SensorConfig limitSwitch(int channel) {
        kind = SensorKind.LIMIT_SWITCH;
        input = InputConfig.create().digital(channel);
        inverted = false;
        return this;
    }

    /**
     * Declares a button with inverted trigger semantics by default.
     *
     * @param channel digital input channel
     * @return this config
     */
    public SensorConfig button(int channel) {
        kind = SensorKind.BUTTON;
        input = InputConfig.create().digital(channel);
        inverted = true;
        return this;
    }

    /**
     * Declares a beam break with inverted trigger semantics by default.
     *
     * @param channel digital input channel
     * @return this config
     */
    public SensorConfig beamBreak(int channel) {
        kind = SensorKind.BEAM_BREAK;
        input = InputConfig.create().digital(channel);
        inverted = true;
        return this;
    }

    /**
     * Sets whether trigger reads should invert raw input values.
     *
     * @param inverted true to invert
     * @return this config
     */
    public SensorConfig inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Marks this sensor as a hardstop.
     *
     * @param direction blocked direction
     * @param position position metadata in mechanism units
     * @return this config
     */
    public SensorConfig hardstop(BlockDirection direction, double position) {
        hardstop = true;
        blockDirection = direction == null ? BlockDirection.NONE : direction;
        this.position = Double.isFinite(position) ? position : 0.0;
        return this;
    }

    /**
     * Lowers this declaration to an immutable spec.
     *
     * @param ownerPath owning path
     * @param name sensor name
     * @return sensor spec
     */
    public SensorSpec toSpec(String ownerPath, String name) {
        if (input == null) {
            throw new IllegalStateException("Sensor input is required for " + ownerPath + "." + name);
        }
        return new SensorSpec(
                ownerPath,
                name,
                kind,
                input.toSpec(ownerPath, name),
                inverted,
                hardstop,
                blockDirection,
                position);
    }
}
