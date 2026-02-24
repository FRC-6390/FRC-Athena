package ca.frc6390.athena.mechanisms;

/**
 * A {@link RegisterableMechanism} that can be built lazily at registration time.
 *
 * <p>This exists so RobotCore configs can store mechanism/superstructure builders (configs) and
 * still register them as part of RobotCore construction without forcing early construction during
 * constants initialization.</p>
 */
public interface RegisterableMechanismFactory extends RegisterableMechanism {
    /**
     * Builds (or returns a cached) registerable mechanism instance.
     */
    RegisterableMechanism build();
}

