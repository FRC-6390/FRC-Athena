package ca.frc6390.athena.core.context;

import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotMechanisms;
import ca.frc6390.athena.networktables.AthenaNT;
import ca.frc6390.athena.networktables.NtScope;

/**
 * Shared context contract for APIs that can access a {@link RobotCore} instance.
 */
public interface RobotScopedContext {
    RobotCore<?> robotCore();

    /**
     * Default robot-core-scoped Athena NetworkTables view.
     */
    default NtScope nt() {
        return AthenaNT.scope("RobotCore/NetworkTables");
    }

    /**
     * Returns the global robot-wide mechanisms view (lookup by name/config/type).
     */
    default RobotMechanisms robotMechanisms() {
        RobotCore<?> core = robotCore();
        if (core == null) {
            throw new IllegalStateException("No RobotCore available in context");
        }
        return core.mechanisms();
    }

    /**
     * Sectioned interaction helper for other already-built mechanisms/superstructures.
     */
    default void robotMechanisms(Consumer<RobotMechanisms.InteractionSection> section) {
        robotMechanisms().use(section);
    }
}
