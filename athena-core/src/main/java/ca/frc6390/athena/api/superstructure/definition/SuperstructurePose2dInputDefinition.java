package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public record SuperstructurePose2dInputDefinition(
    String name,
    Supplier<Pose2d> supplier
) implements SuperstructureInputDefinition {
    public SuperstructurePose2dInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
