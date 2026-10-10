package ca.frc6390.athena.arcp;

import java.util.Arrays;

/**
 * Struct value: WPILib struct bytes plus the schema id that describes them. Use this as the value
 * type for struct signals in {@link ArcpCycle} and {@link WriteHandler}.
 *
 * @param schemaId schema id from {@link ArcpStructSchema#id()}
 * @param bytes WPILib struct layout bytes
 */
public record ArcpStruct(long schemaId, byte[] bytes) {

    public ArcpStruct {
        bytes = bytes == null ? new byte[0] : bytes.clone();
    }

    @Override
    public byte[] bytes() {
        return bytes.clone();
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof ArcpStruct other && other.schemaId == schemaId && Arrays.equals(other.bytes, bytes);
    }

    @Override
    public int hashCode() {
        return 31 * Long.hashCode(schemaId) + Arrays.hashCode(bytes);
    }
}
