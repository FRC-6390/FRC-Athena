package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.mechanism.core.ControlRef;
import java.util.Map;

/**
 * Discovered swerve module with runtime ordering and location.
 *
 * @param name module name
 * @param module module instance
 * @param order module order
 * @param location module location
 * @param explicitLocation true when location came from the module declaration
 * @param controls control refs declared by field name
 */
public record SwerveModuleDefinition(
        String name,
        SwerveModule module,
        int order,
        ModuleLocationRef location,
        boolean explicitLocation,
        Map<String, ControlRef> controls) {
    public SwerveModuleDefinition {
        controls = controls == null ? Map.of() : Map.copyOf(controls);
    }
}
