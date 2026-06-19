package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public record SuperstructureDoubleInputDefinition(
    String name,
    DoubleSupplier supplier
) implements SuperstructureInputDefinition {
    public SuperstructureDoubleInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
