package ca.frc6390.athena.hardware.encoder;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderKind;

/**
 * Immutable normalized encoder declaration.
 *
 * @param ownerPath owning mechanism or subsystem path
 * @param name encoder name
 * @param kind encoder kind
 * @param id device id or channel
 * @param canbus CAN bus name
 * @param signalType primary signal type
 * @param gearRatio mechanism-to-sensor gear ratio
 * @param offset offset in mechanism units
 */
public record EncoderSpec(
        String ownerPath,
        String name,
        EncoderKind kind,
        int id,
        String canbus,
        EncoderSignalType signalType,
        double gearRatio,
        double offset) {
    public EncoderSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "robot" : ownerPath;
        name = name == null || name.isBlank() ? "encoder" : name;
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        signalType = signalType == null ? EncoderSignalType.RELATIVE_POSITION : signalType;
    }

    /**
     * Returns a dotted path useful in validation errors.
     *
     * @return encoder path
     */
    public String path() {
        return ownerPath + "." + name;
    }
}
