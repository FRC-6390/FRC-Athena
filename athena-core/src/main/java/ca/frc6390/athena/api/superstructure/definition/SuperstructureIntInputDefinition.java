package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.IntSupplier;

public record SuperstructureIntInputDefinition(
    String name,
    IntSupplier supplier
) implements SuperstructureInputDefinition {
    public SuperstructureIntInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
