package ca.frc6390.athena.hardware.sensor;

import ca.frc6390.athena.hardware.input.InputSpec;

/**
 * Immutable normalized sensor wrapper declaration.
 *
 * @param ownerPath owning path
 * @param name sensor name
 * @param kind sensor kind
 * @param input input source
 * @param inverted true when trigger semantics are inverted
 * @param hardstop true when a limit switch represents a hardstop
 * @param blockDirection blocked direction
 * @param position position metadata in mechanism units
 */
public record SensorSpec(
        String ownerPath,
        String name,
        SensorKind kind,
        InputSpec input,
        boolean inverted,
        boolean hardstop,
        BlockDirection blockDirection,
        double position) {
    public SensorSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "robot" : ownerPath;
        name = name == null || name.isBlank() ? "sensor" : name;
        if (kind == null) {
            kind = SensorKind.BUTTON;
        }
        if (input == null) {
            throw new IllegalArgumentException("sensor input is required");
        }
        blockDirection = blockDirection == null ? BlockDirection.NONE : blockDirection;
        position = Double.isFinite(position) ? position : 0.0;
    }

    /**
     * Returns a dotted path useful in diagnostics and validation errors.
     *
     * @return sensor path
     */
    public String path() {
        return ownerPath + "." + name;
    }

    /**
     * Applies configured inversion to a raw boolean read.
     *
     * @param raw raw input value
     * @return trigger state
     */
    public boolean triggered(boolean raw) {
        return inverted != raw;
    }
}
