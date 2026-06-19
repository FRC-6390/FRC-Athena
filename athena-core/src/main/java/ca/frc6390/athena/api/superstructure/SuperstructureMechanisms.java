package ca.frc6390.athena.api.superstructure;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.api.mechanism.MechanismConfig;
import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.superstructure.definition.ExistingMechanismChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.ExistingNestedSuperstructureChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.MechanismChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.NestedSuperstructureChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;

public final class SuperstructureMechanisms<SP> {
    private final List<SuperstructureChildDefinition<SP>> definitions = new ArrayList<>();

    private SuperstructureMechanisms() {
    }

    public static <SP> SuperstructureMechanisms<SP> create() {
        return new SuperstructureMechanisms<>();
    }

    public static <SP> SuperstructureMechanisms<SP> from(List<SuperstructureChildDefinition<SP>> definitions) {
        SuperstructureMechanisms<SP> mechanisms = create();
        if (definitions != null) {
            mechanisms.definitions.addAll(definitions);
        }
        return mechanisms;
    }

    public <E> SuperstructureMechanisms<SP> mechanism(
            MechanismDefinition definition,
            Function<SP, E> mapper) {
        definitions.add(new MechanismChildDefinition<>(Objects.requireNonNull(definition, "definition"),
            Objects.requireNonNull(mapper, "mapper")));
        return this;
    }

    public <E> SuperstructureMechanisms<SP> mechanism(
            Class<?> declarationType,
            Function<SP, E> mapper) {
        return mechanism(MechanismDefinitions.structured(Objects.requireNonNull(declarationType, "declarationType")),
            mapper);
    }

    public <E> SuperstructureMechanisms<SP> mechanism(
            TypedStatefulMechanismConfig<E> declaration,
            Function<SP, E> mapper) {
        return mechanism(Objects.requireNonNull(declaration, "declaration").definition(), mapper);
    }

    public <E> SuperstructureMechanisms<SP> existing(
            StatefulMechanism<E> mechanism,
            Function<SP, E> mapper) {
        definitions.add(new ExistingMechanismChildDefinition<>(Objects.requireNonNull(mechanism, "mechanism"),
            Objects.requireNonNull(mapper, "mapper")));
        return this;
    }

    public <CS, CSP> SuperstructureMechanisms<SP> superstructure(
            SuperstructureDefinition<CSP> definition,
            Function<SP, CS> mapper) {
        definitions.add(new NestedSuperstructureChildDefinition<>(Objects.requireNonNull(definition, "definition"),
            Objects.requireNonNull(mapper, "mapper")));
        return this;
    }

    public <CS, CSP> SuperstructureMechanisms<SP> superstructure(
            Class<?> declarationType,
            Function<SP, CS> mapper) {
        return superstructure(SuperstructureDefinitions.structured(
            Objects.requireNonNull(declarationType, "declarationType")), mapper);
    }

    public <CS, CSP> SuperstructureMechanisms<SP> superstructure(
            SuperstructureConfig<CS, CSP> declaration,
            Function<SP, CS> mapper) {
        return superstructure(Objects.requireNonNull(declaration, "declaration").definition(), mapper);
    }

    public <CS, CSP> SuperstructureMechanisms<SP> existingSuperstructure(
            SuperstructureMechanism<CS, CSP> superstructure,
            Function<SP, CS> mapper) {
        definitions.add(new ExistingNestedSuperstructureChildDefinition<>(
            Objects.requireNonNull(superstructure, "superstructure"),
            Objects.requireNonNull(mapper, "mapper")));
        return this;
    }

    public SuperstructureMechanisms<SP> merge(SuperstructureMechanisms<SP> other) {
        if (other != null) {
            definitions.addAll(other.definitions);
        }
        return this;
    }

    public List<SuperstructureChildDefinition<SP>> definitions() {
        return List.copyOf(definitions);
    }
}
