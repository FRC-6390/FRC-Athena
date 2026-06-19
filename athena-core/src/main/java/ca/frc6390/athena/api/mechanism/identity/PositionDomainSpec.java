package ca.frc6390.athena.api.mechanism.identity;

import java.util.Objects;

public record PositionDomainSpec(
    PositionDomainKind kind,
    PositionUnit units
) {
    public PositionDomainSpec {
        kind = Objects.requireNonNull(kind, "kind");
        units = Objects.requireNonNull(units, "units");
    }
}
