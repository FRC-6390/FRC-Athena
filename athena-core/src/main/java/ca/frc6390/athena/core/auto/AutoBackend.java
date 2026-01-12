package ca.frc6390.athena.core.auto;

import java.util.Optional;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotAuto;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Service-loaded adapter implemented by vendor modules to wire localization and autonomous
 * features without pulling vendor dependencies into the core.
 */
public interface AutoBackend {
    /**
        * @return true if this backend can serve the requested auto source.
        */
    boolean supports(RobotAuto.AutoSource source);

    /**
        * Configure holonomic path following for the given bindings. Returns true if successful.
        */
    default boolean configureHolonomic(HolonomicDriveBinding binding) {
        return false;
    }

    /**
        * @return a warmup command for the given source (if any).
        */
    default Optional<Command> warmupCommand(RobotAuto.AutoSource source) {
        return Optional.empty();
    }

    /**
        * Register a named command for auto construction (PathPlanner style).
        */
    default boolean registerNamedCommand(String id, Supplier<Command> supplier) {
        return false;
    }

    /**
        * Build an autonomous command for the requested source, if supported.
        */
    default Optional<Command> buildAuto(RobotAuto.AutoSource source, String reference) {
        return Optional.empty();
    }

    /**
        * Create a Choreo-style auto factory if supported.
        */
    default Optional<ChoreoAutoFactory> createChoreoFactory(ChoreoBinding binding) {
        return Optional.empty();
    }
}
