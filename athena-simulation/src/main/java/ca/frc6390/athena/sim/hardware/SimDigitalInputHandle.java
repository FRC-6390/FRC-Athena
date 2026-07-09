package ca.frc6390.athena.sim.hardware;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import java.util.Objects;

/**
 * In-memory simulation state for a digital input declaration.
 */
public final class SimDigitalInputHandle {
    private final DigitalInputDevice device;
    private boolean value;

    /**
     * Creates a simulated digital input.
     *
     * @param device input declaration
     */
    public SimDigitalInputHandle(DigitalInputDevice device) {
        this.device = Objects.requireNonNull(device, "device");
    }

    /**
     * Returns the input declaration.
     *
     * @return device
     */
    public DigitalInputDevice device() {
        return device;
    }

    /**
     * Reads the raw simulated input value.
     *
     * @return raw value
     */
    public boolean raw() {
        return value;
    }

    /**
     * Sets the raw simulated input value.
     *
     * @param value raw value
     * @return this handle
     */
    public SimDigitalInputHandle raw(boolean value) {
        this.value = value;
        return this;
    }
}
