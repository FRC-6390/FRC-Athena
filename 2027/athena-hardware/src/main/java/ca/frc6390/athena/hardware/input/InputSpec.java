package ca.frc6390.athena.hardware.input;

import java.util.Objects;

/**
 * Immutable normalized input declaration.
 *
 * @param ownerPath owning mechanism or subsystem path
 * @param name input name inside the owner
 * @param type input value type
 * @param sourceKind source kind
 * @param channel hardware channel, or {@code -1} for non-channel sources
 * @param label source label or constant text
 */
public record InputSpec(
        String ownerPath,
        String name,
        InputType type,
        InputSourceKind sourceKind,
        int channel,
        String label) {
    public InputSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "robot" : ownerPath;
        name = name == null || name.isBlank() ? "input" : name;
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(sourceKind, "sourceKind");
        label = label == null ? "" : label;
    }

    /**
     * Returns a dotted path useful in validation errors.
     *
     * @return input path
     */
    public String path() {
        return ownerPath + "." + name;
    }
}
