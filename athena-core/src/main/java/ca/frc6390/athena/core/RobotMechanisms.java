package ca.frc6390.athena.core;

import java.util.AbstractMap;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import ca.frc6390.athena.mechanisms.ArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.SuperstructureConfig;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.TurretMechanism;

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

    public Mechanism byConfig(MechanismConfig<?> config) {
        if (config == null) {
            return null;
        }
        for (Mechanism mech : mechanisms.values()) {
            if (mech != null && mech.getSourceConfig() == config) {
                return mech;
            }
        }
        return null;
    }

    public SuperstructureMechanism<?, ?> byConfig(SuperstructureConfig<?, ?> config) {
        if (config == null) {
            return null;
        }
        for (SuperstructureMechanism<?, ?> superstructure : superstructuresByConfig) {
            if (superstructure != null && superstructure.getSourceConfig() == config) {
                return superstructure;
            }
        }
        return null;
    }

    public SuperstructureMechanism<?, ?> superstruct(String name) {
        return superstructuresByName.get(name);
    }

    public SuperstructureMechanism<?, ?> superstruct(Enum<?> key) {
        Objects.requireNonNull(key, "key");
        return superstruct(key.name());
    }

    public SuperstructureMechanism<?, ?> superstruct(SuperstructureConfig<?, ?> config) {
        return byConfig(config);
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

    public <T> T key(Enum<?> key, Class<T> type) {
        Objects.requireNonNull(key, "key");
        return key(key.name(), type);
    }

    public <T> T key(MechanismConfig<?> config, Class<T> type) {
        Objects.requireNonNull(type, "type");
        Mechanism mech = byConfig(config);
        if (mech == null) {
            return null;
        }
        if (!type.isInstance(mech)) {
            throw new IllegalArgumentException("Mechanism built from config is not a " + type.getSimpleName()
                    + " (was " + mech.getClass().getSimpleName() + ")");
        }
        return type.cast(mech);
    }

    public <T> T key(SuperstructureConfig<?, ?> config, Class<T> type) {
        Objects.requireNonNull(type, "type");
        SuperstructureMechanism<?, ?> superstructure = byConfig(config);
        if (superstructure == null) {
            return null;
        }
        if (!type.isInstance(superstructure)) {
            throw new IllegalArgumentException("Superstructure built from config is not a " + type.getSimpleName()
                    + " (was " + superstructure.getClass().getSimpleName() + ")");
        }
        return type.cast(superstructure);
    }

    public TurretMechanism turret(String name) {
        return key(name, TurretMechanism.class);
    }

    public TurretMechanism turret(Enum<?> key) {
        return key(key, TurretMechanism.class);
    }

    public TurretMechanism turret(MechanismConfig<?> config) {
        return key(config, TurretMechanism.class);
    }

    public ElevatorMechanism elevator(String name) {
        return key(name, ElevatorMechanism.class);
    }

    public ElevatorMechanism elevator(Enum<?> key) {
        return key(key, ElevatorMechanism.class);
    }

    public ElevatorMechanism elevator(MechanismConfig<?> config) {
        return key(config, ElevatorMechanism.class);
    }

    public ArmMechanism arm(String name) {
        return key(name, ArmMechanism.class);
    }

    public ArmMechanism arm(Enum<?> key) {
        return key(key, ArmMechanism.class);
    }

    public ArmMechanism arm(MechanismConfig<?> config) {
        return key(config, ArmMechanism.class);
    }

    public FlywheelMechanism flywheel(String name) {
        return key(name, FlywheelMechanism.class);
    }

    public FlywheelMechanism flywheel(Enum<?> key) {
        return key(key, FlywheelMechanism.class);
    }

    public FlywheelMechanism flywheel(MechanismConfig<?> config) {
        return key(config, FlywheelMechanism.class);
    }

    public Mechanism generic(String name) {
        return key(name, Mechanism.class);
    }

    public Mechanism generic(Enum<?> key) {
        return key(key, Mechanism.class);
    }

    public Mechanism generic(MechanismConfig<?> config) {
        return key(config, Mechanism.class);
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
}
