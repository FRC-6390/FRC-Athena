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
 * @param inverted true when sensor direction should be inverted
 * @param gearRatio mechanism-to-sensor gear ratio
 * @param conversion mechanism-unit conversion factor
 * @param offset offset in mechanism units
 * @param units configured encoder units
 */
public record EncoderSpec(
        String ownerPath,
        String name,
        EncoderKind kind,
        int id,
        String canbus,
        EncoderSignalType signalType,
        boolean inverted,
        double gearRatio,
        double conversion,
        double offset,
        EncoderUnit units) {
    public EncoderSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "robot" : ownerPath;
        name = name == null || name.isBlank() ? "encoder" : name;
        Objects.requireNonNull(kind, "kind");
        canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
        signalType = signalType == null ? EncoderSignalType.RELATIVE_POSITION : signalType;
        units = units == null ? EncoderUnit.RAW : units;
    }

    public EncoderSpec(
            String ownerPath,
            String name,
            EncoderKind kind,
            int id,
            String canbus,
            EncoderSignalType signalType,
            boolean inverted,
            double gearRatio,
            double conversion,
            double offset) {
        this(ownerPath, name, kind, id, canbus, signalType, inverted, gearRatio, conversion, offset, EncoderUnit.RAW);
    }

    /**
     * Creates an encoder spec with default direction and conversion.
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
    public EncoderSpec(
            String ownerPath,
            String name,
            EncoderKind kind,
            int id,
            String canbus,
            EncoderSignalType signalType,
            double gearRatio,
            double offset) {
        this(ownerPath, name, kind, id, canbus, signalType, false, gearRatio, 1.0, offset);
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
