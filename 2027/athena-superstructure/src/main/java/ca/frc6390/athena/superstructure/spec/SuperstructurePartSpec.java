package ca.frc6390.athena.superstructure.spec;

import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.mechanism.spec.MechanismSpec;

/**
 * Named child part within a superstructure.
 *
 * @param name part name
 * @param kind child kind
 * @param mechanism mechanism spec, when this part wraps a mechanism
 * @param superstructure nested superstructure spec, when this part wraps a
 *        superstructure
 */
public record SuperstructurePartSpec(
        String name,
        SuperstructurePartSpec.Kind kind,
        MechanismSpec mechanism,
        SuperstructureSpec superstructure) {
    /**
     * Superstructure child kinds.
     */
    public enum Kind {
        /** Mechanism child part. */
        MECHANISM,

        /** Nested superstructure child part. */
        SUPERSTRUCTURE
    }

    public SuperstructurePartSpec {
        name = name == null || name.isBlank() ? "part" : name;
        kind = Objects.requireNonNull(kind, "kind");
        if (kind == Kind.MECHANISM) {
            Objects.requireNonNull(mechanism, "mechanism");
        } else {
            Objects.requireNonNull(superstructure, "superstructure");
        }
    }

    /**
     * Creates a mechanism-backed part.
     *
     * @param name part name
     * @param mechanism mechanism spec
     */
    public SuperstructurePartSpec(String name, MechanismSpec mechanism) {
        this(name, Kind.MECHANISM, mechanism, null);
    }

    /**
     * Creates a nested superstructure-backed part.
     *
     * @param name part name
     * @param superstructure superstructure spec
     */
    public SuperstructurePartSpec(String name, SuperstructureSpec superstructure) {
        this(name, Kind.SUPERSTRUCTURE, null, superstructure);
    }

    /**
     * Returns the mechanism spec when this is a mechanism part.
     *
     * @return optional mechanism spec
     */
    public Optional<MechanismSpec> mechanismPart() {
        return Optional.ofNullable(mechanism);
    }

    /**
     * Returns the nested superstructure spec when this is a superstructure part.
     *
     * @return optional nested superstructure spec
     */
    public Optional<SuperstructureSpec> superstructurePart() {
        return Optional.ofNullable(superstructure);
    }
}
