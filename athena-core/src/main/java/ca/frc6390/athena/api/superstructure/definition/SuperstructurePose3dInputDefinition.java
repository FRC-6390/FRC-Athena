package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;

public record SuperstructurePose3dInputDefinition(
    String name,
    Supplier<Pose3d> supplier
) implements SuperstructureInputDefinition {
    public SuperstructurePose3dInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
