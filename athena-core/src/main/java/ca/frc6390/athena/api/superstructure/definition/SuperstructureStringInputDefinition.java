package ca.frc6390.athena.api.superstructure.definition;

import java.util.Objects;
import java.util.function.Supplier;

public record SuperstructureStringInputDefinition(
    String name,
    Supplier<String> supplier
) implements SuperstructureInputDefinition {
    public SuperstructureStringInputDefinition {
        name = Objects.requireNonNull(name, "name");
        supplier = Objects.requireNonNull(supplier, "supplier");
    }
}
