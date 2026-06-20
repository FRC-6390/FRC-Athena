package ca.frc6390.athena.hardware.input;

/**
 * Student-facing generic input declaration.
 */
public final class InputConfig {
    private InputType type;
    private InputSourceKind sourceKind;
    private int channel = -1;
    private String label = "";

    private InputConfig() {
    }

    /**
     * Creates an input config.
     *
     * @return input config
     */
    public static InputConfig create() {
        return new InputConfig();
    }

    /**
     * Declares a digital boolean input.
     *
     * @param channel digital input channel
     * @return this config
     */
    public InputConfig digital(int channel) {
        type = InputType.BOOLEAN;
        sourceKind = InputSourceKind.DIGITAL_CHANNEL;
        this.channel = channel;
        return this;
    }

    /**
     * Declares an analog numeric input.
     *
     * @param channel analog input channel
     * @return this config
     */
    public InputConfig analog(int channel) {
        type = InputType.NUMBER;
        sourceKind = InputSourceKind.ANALOG_CHANNEL;
        this.channel = channel;
        return this;
    }

    /**
     * Declares a runtime-supplied boolean input.
     *
     * @param label source label
     * @return this config
     */
    public InputConfig runtimeBoolean(String label) {
        type = InputType.BOOLEAN;
        sourceKind = InputSourceKind.RUNTIME_SUPPLIER;
        this.label = label;
        return this;
    }

    /**
     * Declares a runtime-supplied numeric input.
     *
     * @param label source label
     * @return this config
     */
    public InputConfig runtimeNumber(String label) {
        type = InputType.NUMBER;
        sourceKind = InputSourceKind.RUNTIME_SUPPLIER;
        this.label = label;
        return this;
    }

    /**
     * Declares a runtime-supplied string input.
     *
     * @param label source label
     * @return this config
     */
    public InputConfig runtimeString(String label) {
        type = InputType.STRING;
        sourceKind = InputSourceKind.RUNTIME_SUPPLIER;
        this.label = label;
        return this;
    }

    /**
     * Declares a constant boolean input.
     *
     * @param value value
     * @return this config
     */
    public InputConfig constant(boolean value) {
        type = InputType.BOOLEAN;
        sourceKind = InputSourceKind.CONSTANT;
        label = Boolean.toString(value);
        return this;
    }

    /**
     * Declares a constant numeric input.
     *
     * @param value value
     * @return this config
     */
    public InputConfig constant(double value) {
        type = InputType.NUMBER;
        sourceKind = InputSourceKind.CONSTANT;
        label = Double.toString(value);
        return this;
    }

    /**
     * Declares a constant string input.
     *
     * @param value value
     * @return this config
     */
    public InputConfig constant(String value) {
        type = InputType.STRING;
        sourceKind = InputSourceKind.CONSTANT;
        label = value;
        return this;
    }

    /**
     * Lowers this declaration to an immutable spec.
     *
     * @param ownerPath owning path
     * @param name input name
     * @return input spec
     */
    public InputSpec toSpec(String ownerPath, String name) {
        if (type == null || sourceKind == null) {
            throw new IllegalStateException("Input source is required for " + ownerPath + "." + name);
        }
        return new InputSpec(ownerPath, name, type, sourceKind, channel, label);
    }
}
