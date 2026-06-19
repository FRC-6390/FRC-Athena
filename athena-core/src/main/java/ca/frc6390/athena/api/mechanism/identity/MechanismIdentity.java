package ca.frc6390.athena.api.mechanism.identity;

import java.util.Objects;
import java.util.Optional;

public record MechanismIdentity(
    Optional<PositionDomainSpec> positionDomain,
    Optional<TravelRange> travelRange,
    boolean continuousRotation,
    boolean disabled
) {
    public MechanismIdentity {
        positionDomain = Objects.requireNonNull(positionDomain, "positionDomain");
        travelRange = Objects.requireNonNull(travelRange, "travelRange");
    }

    public static MechanismIdentity empty() {
        return new MechanismIdentity(Optional.empty(), Optional.empty(), false, false);
    }
}
