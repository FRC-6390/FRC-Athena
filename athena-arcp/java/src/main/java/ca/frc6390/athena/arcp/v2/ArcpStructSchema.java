package ca.frc6390.athena.arcp.v2;

/**
 * WPILib struct schema announced in a STRUCT_SCHEMA record.
 *
 * @param id schema id referenced by struct signals
 * @param typeName WPILib type name, for example {@code Pose2d}
 * @param schema WPILib struct schema string
 */
public record ArcpStructSchema(long id, String typeName, String schema) {
}
