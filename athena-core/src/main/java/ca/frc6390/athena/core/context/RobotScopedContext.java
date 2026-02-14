package ca.frc6390.athena.core.context;

import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotMechanisms;

/**
 * Shared context contract for APIs that can access a {@link RobotCore} instance.
 */
public interface RobotScopedContext {
    RobotCore<?> robotCore();

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
