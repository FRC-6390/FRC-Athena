package ca.frc6390.athena.mechanism.persist;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Robot-defined conditions for tuning lock groups. Enforcement happens in the robot runtime's write
 * path; clients never decide whether a write is allowed.
 */
public final class TuningLocks {
    private final Map<Class<?>, BooleanSupplier> conditions = new HashMap<>();

    /** Creates a registry with the built-in groups: {@code InMatch} unlocked until defined, {@code Never}, {@code Always}. */
    public TuningLocks() {
        conditions.put(Locks.InMatch.class, () -> false);
        conditions.put(Locks.Never.class, () -> false);
        conditions.put(Locks.Always.class, () -> true);
    }

    /**
     * Defines the locked condition of a lock group. Redefining a group replaces its condition.
     *
     * @param group lock group class
     * @param locked true while writes to the group must be rejected
     * @return this registry
     */
    public TuningLocks define(Class<?> group, BooleanSupplier locked) {
        Objects.requireNonNull(group, "group");
        Objects.requireNonNull(locked, "locked");
        if (group == Locks.Never.class || group == Locks.Always.class) {
            throw new IllegalArgumentException("Built-in lock group " + group.getSimpleName() + " cannot be redefined.");
        }
        conditions.put(group, locked);
        return this;
    }

    /** Returns true when the group has a condition defined by {@link #define}. */
    public boolean defined(Class<?> group) {
        return group != null && conditions.containsKey(group);
    }

    /**
     * Returns whether writes to the group are currently rejected. Groups without a definition behave
     * as {@link Locks.InMatch}.
     *
     * @param group lock group, or null for no lock
     * @return true while locked
     */
    public boolean isLocked(Class<?> group) {
        if (group == null) {
            return false;
        }
        BooleanSupplier condition = conditions.get(group);
        if (condition == null) {
            condition = conditions.get(Locks.InMatch.class);
        }
        return condition.getAsBoolean();
    }

    /**
     * Returns a live view of the group's lock state. The view re-reads the condition on every call,
     * so definitions made after the schema is built still apply.
     *
     * @param group lock group
     * @return lock condition
     */
    public BooleanSupplier lockCondition(Class<?> group) {
        Objects.requireNonNull(group, "group");
        return () -> isLocked(group);
    }
}
