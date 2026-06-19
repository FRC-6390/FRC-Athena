package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.BooleanSupplier;

public record SuperstructureBooleanInputDefinition(
    String name,
    BooleanSupplier supplier
) implements SuperstructureInputDefinition {
    public SuperstructureBooleanInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
