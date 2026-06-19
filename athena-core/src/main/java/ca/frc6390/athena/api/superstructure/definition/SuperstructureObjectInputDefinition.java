package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Supplier;

public record SuperstructureObjectInputDefinition(
    String name,
    Supplier<?> supplier
) implements SuperstructureInputDefinition {
    public SuperstructureObjectInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
