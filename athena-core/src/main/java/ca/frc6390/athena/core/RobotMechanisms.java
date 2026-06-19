package ca.frc6390.athena.core;

import java.util.AbstractMap;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;

import ca.frc6390.athena.api.ConfigDeclarations;
import ca.frc6390.athena.api.mechanism.MechanismConfig;
import ca.frc6390.athena.mechanisms.ArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.TurretMechanism;
import ca.frc6390.athena.mechanisms.statespec.StateNames;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.superstructure.SuperstructureConfig;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;

/**
 * Read-only view over RobotCore's registered mechanisms with convenience lookup helpers.
 *
 * <p>This implements {@link Map} so existing code that treated {@code getMechanisms()} as a map
 * can keep working, while allowing richer access patterns like {@code byName(...)} and
 * {@code turret("HoodTurret")}.</p>
 */
public final class RobotMechanisms extends AbstractMap<String, Mechanism> {
    private final Map<String, Mechanism> mechanisms;
    private final Map<String, SuperstructureMechanism<?, ?>> superstructuresByName;
    private final List<SuperstructureMechanism<?, ?>> superstructuresByConfig;

    RobotMechanisms(Map<String, Mechanism> mechanisms,
                    Map<String, SuperstructureMechanism<?, ?>> superstructuresByName,
                    List<SuperstructureMechanism<?, ?>> superstructuresByConfig) {
        this.mechanisms = Objects.requireNonNull(mechanisms, "mechanisms");
        this.superstructuresByName = Objects.requireNonNull(superstructuresByName, "superstructuresByName");
        this.superstructuresByConfig = Objects.requireNonNull(superstructuresByConfig, "superstructuresByConfig");
    }

    public Mechanism byName(String name) {
        return get(name);
    }

    /**
     * Sectioned runtime interaction API for already-built mechanisms/superstructures.
     *
     * <p>Unlike config builders, this operates on live registered instances and is intended for
     * "do work if present" patterns from hooks, autos, and command orchestration.</p>
     */
    public RobotMechanisms use(Consumer<InteractionSection> section) {
        if (section != null) {
            section.accept(new InteractionSection(this));
        }
        return this;
    }

    public Optional<Mechanism> findByName(String name) {
        return Optional.ofNullable(get(name));
    }

    public Mechanism requireByName(String name) {
        Mechanism mech = get(name);
        if (mech == null) {
            throw new IllegalArgumentException("No mechanism registered with name '" + name + "'");
        }
        return mech;
    }

    public <M extends Mechanism> M byName(String name, Class<M> type) {
        Mechanism mech = get(name);
        if (mech == null) {
            return null;
        }
        if (!type.isInstance(mech)) {
            throw new IllegalArgumentException("Mechanism '" + name + "' is not a " + type.getSimpleName()
                    + " (was " + mech.getClass().getSimpleName() + ")");
        }
        return type.cast(mech);
    }

    public Mechanism bySource(Object source) {
        if (source == null) {
            return null;
        }
        for (Mechanism mech : mechanisms.values()) {
            if (mech != null && mech.getSourceKey() == source) {
                return mech;
            }
        }
        return null;
    }

    @SuppressWarnings("unchecked")
    public <SP, S> SuperstructureMechanism<S, SP> bySuperstructureSource(Object source) {
        if (source == null) {
            return null;
        }
        for (SuperstructureMechanism<?, ?> superstructure : superstructuresByConfig) {
            if (superstructure != null && superstructure.getSourceKey() == source) {
                return (SuperstructureMechanism<S, SP>) superstructure;
            }
        }
        return null;
    }

    public SuperstructureMechanism<?, ?> superstruct(String name) {
        return superstructuresByName.get(name);
    }

    public SuperstructureMechanism<?, ?> superstruct(Object key) {
        Objects.requireNonNull(key, "key");
        return superstruct(StateNames.name(key));
    }

    public <SP, S> SuperstructureMechanism<S, SP> superstruct(
            SuperstructureDefinition<?> definition) {
        return key(definition, SuperstructureMechanism.class);
    }

    @SuppressWarnings("unchecked")
    public <SP, S> SuperstructureMechanism<S, SP> superstruct(
            Class<?> declarationType) {
        return (SuperstructureMechanism<S, SP>) key(declarationType, SuperstructureMechanism.class);
    }

    public <T> T key(String name, Class<T> type) {
        Objects.requireNonNull(type, "type");
        if (name == null || name.isBlank()) {
            return null;
        }
        if (SuperstructureMechanism.class.isAssignableFrom(type)) {
            Object obj = superstructuresByName.get(name);
            if (obj == null) {
                return null;
            }
            if (!type.isInstance(obj)) {
                throw new IllegalArgumentException("Superstructure '" + name + "' is not a " + type.getSimpleName()
                        + " (was " + obj.getClass().getSimpleName() + ")");
            }
            return type.cast(obj);
        }
        Object obj = mechanisms.get(name);
        if (obj == null) {
            return null;
        }
        if (!type.isInstance(obj)) {
            throw new IllegalArgumentException("Mechanism '" + name + "' is not a " + type.getSimpleName()
                    + " (was " + obj.getClass().getSimpleName() + ")");
        }
        return type.cast(obj);
    }

    public <T> T key(Object key, Class<T> type) {
        Objects.requireNonNull(key, "key");
        return key(StateNames.name(key), type);
    }

    public <T> T key(MechanismDefinition definition, Class<T> type) {
        Objects.requireNonNull(type, "type");
        Mechanism mech = bySource(definition);
        if (mech == null) {
            return null;
        }
        if (!type.isInstance(mech)) {
            throw new IllegalArgumentException("Mechanism built from definition is not a " + type.getSimpleName()
                    + " (was " + mech.getClass().getSimpleName() + ")");
        }
        return type.cast(mech);
    }

    public <T> T key(SuperstructureDefinition<?> definition, Class<T> type) {
        Objects.requireNonNull(type, "type");
        SuperstructureMechanism<?, ?> superstructure = bySuperstructureSource(definition);
        if (superstructure == null) {
            return null;
        }
        if (!type.isInstance(superstructure)) {
            throw new IllegalArgumentException("Superstructure built from definition is not a " + type.getSimpleName()
                    + " (was " + superstructure.getClass().getSimpleName() + ")");
        }
        return type.cast(superstructure);
    }

    public <T> T key(Class<?> declarationType, Class<T> type) {
        Objects.requireNonNull(declarationType, "declarationType");
        if (MechanismConfig.class.isAssignableFrom(declarationType)) {
            MechanismConfig declaration = ConfigDeclarations.instance(declarationType.asSubclass(MechanismConfig.class));
            return key(declaration.name(), type);
        }
        if (SuperstructureConfig.class.isAssignableFrom(declarationType)) {
            SuperstructureConfig<?, ?> declaration =
                ConfigDeclarations.instance(declarationType.asSubclass(SuperstructureConfig.class));
            return key(declaration.name(), type);
        }
        RuntimeException mechanismFailure;
        try {
            return key(ca.frc6390.athena.api.mechanism.MechanismDefinitions.structured(declarationType).name(), type);
        } catch (RuntimeException ex) {
            mechanismFailure = ex;
        }
        try {
            return key(ca.frc6390.athena.api.superstructure.SuperstructureDefinitions.structured(declarationType).name(), type);
        } catch (RuntimeException ex) {
            IllegalArgumentException failure = new IllegalArgumentException(
                "Declaration type is not a mechanism or superstructure config: " + declarationType.getName(), ex);
            failure.addSuppressed(mechanismFailure);
            throw failure;
        }
    }

    public TurretMechanism turret(String name) {
        return key(name, TurretMechanism.class);
    }

    public TurretMechanism turret(Object key) {
        return key(key, TurretMechanism.class);
    }

    public TurretMechanism turret(MechanismDefinition definition) {
        return key(definition, TurretMechanism.class);
    }

    public ElevatorMechanism elevator(String name) {
        return key(name, ElevatorMechanism.class);
    }

    public ElevatorMechanism elevator(Object key) {
        return key(key, ElevatorMechanism.class);
    }

    public ElevatorMechanism elevator(MechanismDefinition definition) {
        return key(definition, ElevatorMechanism.class);
    }

    public ArmMechanism arm(String name) {
        return key(name, ArmMechanism.class);
    }

    public ArmMechanism arm(Object key) {
        return key(key, ArmMechanism.class);
    }

    public ArmMechanism arm(MechanismDefinition definition) {
        return key(definition, ArmMechanism.class);
    }

    public FlywheelMechanism flywheel(String name) {
        return key(name, FlywheelMechanism.class);
    }

    public FlywheelMechanism flywheel(Object key) {
        return key(key, FlywheelMechanism.class);
    }

    public FlywheelMechanism flywheel(MechanismDefinition definition) {
        return key(definition, FlywheelMechanism.class);
    }

    public Mechanism generic(String name) {
        return key(name, Mechanism.class);
    }

    public Mechanism generic(Object key) {
        return key(key, Mechanism.class);
    }

    public Mechanism generic(MechanismDefinition definition) {
        return key(definition, Mechanism.class);
    }

    @Override
    public Set<Entry<String, Mechanism>> entrySet() {
        return Collections.unmodifiableMap(mechanisms).entrySet();
    }

    @Override
    public int size() {
        return mechanisms.size();
    }

    @Override
    public boolean isEmpty() {
        return mechanisms.isEmpty();
    }

    @Override
    public boolean containsKey(Object key) {
        return mechanisms.containsKey(key);
    }

    @Override
    public boolean containsValue(Object value) {
        return mechanisms.containsValue(value);
    }

    @Override
    public Mechanism get(Object key) {
        return mechanisms.get(key);
    }

    @Override
    public Collection<Mechanism> values() {
        return Collections.unmodifiableCollection(mechanisms.values());
    }

    public static final class InteractionSection {
        private final RobotMechanisms owner;

        private InteractionSection(RobotMechanisms owner) {
            this.owner = owner;
        }

        public InteractionSection mechanism(String name, Consumer<Mechanism> action) {
            Mechanism mechanism = owner.byName(name);
            if (mechanism != null && action != null) {
                action.accept(mechanism);
            }
            return this;
        }

        public <M extends Mechanism> InteractionSection mechanism(
                String name,
                Class<M> type,
                Consumer<M> action) {
            M mechanism = owner.byName(name, type);
            if (mechanism != null && action != null) {
                action.accept(mechanism);
            }
            return this;
        }

        public InteractionSection mechanism(MechanismDefinition definition, Consumer<Mechanism> action) {
            Mechanism mechanism = owner.generic(definition);
            if (mechanism != null && action != null) {
                action.accept(mechanism);
            }
            return this;
        }

        public <M extends Mechanism> InteractionSection mechanism(
                MechanismDefinition definition,
                Class<M> type,
                Consumer<M> action) {
            M mechanism = owner.key(definition, type);
            if (mechanism != null && action != null) {
                action.accept(mechanism);
            }
            return this;
        }

        public InteractionSection requireMechanism(String name, Consumer<Mechanism> action) {
            Mechanism mechanism = owner.requireByName(name);
            if (action != null) {
                action.accept(mechanism);
            }
            return this;
        }

        public <M extends Mechanism> InteractionSection requireMechanism(
                String name,
                Class<M> type,
                Consumer<M> action) {
            Mechanism mechanism = owner.requireByName(name);
            if (!type.isInstance(mechanism)) {
                throw new IllegalArgumentException("Mechanism '" + name + "' is not a "
                        + type.getSimpleName() + " (was " + mechanism.getClass().getSimpleName() + ")");
            }
            if (action != null) {
                action.accept(type.cast(mechanism));
            }
            return this;
        }

        public InteractionSection superstructure(
                String name,
                Consumer<SuperstructureMechanism<?, ?>> action) {
            SuperstructureMechanism<?, ?> superstructure = owner.superstruct(name);
            if (superstructure != null && action != null) {
                action.accept(superstructure);
            }
            return this;
        }

        public <SP, S> InteractionSection superstructure(
                SuperstructureDefinition<?> definition,
                Consumer<SuperstructureMechanism<S, SP>> action) {
            SuperstructureMechanism<S, SP> superstructure = owner.key(definition, SuperstructureMechanism.class);
            if (superstructure != null && action != null) {
                action.accept(superstructure);
            }
            return this;
        }
    }
}
