package ca.frc6390.athena.api.mechanism.identity;

import java.util.Optional;

public final class IdentityConfig {
    private PositionDomainSpec positionDomain;
    private TravelRange travelRange;
    private boolean continuousRotation;
    private boolean disabled;

    private IdentityConfig() {
    }

    public static IdentityConfig create() {
        return new IdentityConfig();
    }

    public static IdentityConfig from(MechanismIdentity identity) {
        IdentityConfig config = create();
        identity.positionDomain().ifPresent(domain -> config.positionDomain = domain);
        identity.travelRange().ifPresent(range -> config.travelRange = range);
        config.continuousRotation = identity.continuousRotation();
        config.disabled = identity.disabled();
        return config;
    }

    public IdentityConfig positionDomain(PositionDomainKind kind, PositionUnit units) {
        this.positionDomain = new PositionDomainSpec(kind, units);
        return this;
    }

    public IdentityConfig travelRange(double min, double max) {
        this.travelRange = new TravelRange(min, max);
        return this;
    }

    public IdentityConfig continuousRotation() {
        this.continuousRotation = true;
        return this;
    }

    public IdentityConfig disabled(boolean disabled) {
        this.disabled = disabled;
        return this;
    }

    public IdentityConfig merge(IdentityConfig other) {
        if (other == null) {
            return this;
        }
        if (other.positionDomain != null) {
            this.positionDomain = other.positionDomain;
        }
        if (other.travelRange != null) {
            this.travelRange = other.travelRange;
        }
        this.continuousRotation = this.continuousRotation || other.continuousRotation;
        this.disabled = this.disabled || other.disabled;
        return this;
    }

    public MechanismIdentity definition() {
        return new MechanismIdentity(
            Optional.ofNullable(positionDomain),
            Optional.ofNullable(travelRange),
            continuousRotation,
            disabled);
    }
}
