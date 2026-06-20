package ca.frc6390.athena.examples;

import ca.frc6390.athena.superstructure.config.SuperstructureConfig;
import ca.frc6390.athena.superstructure.config.Superstructures;

/**
 * Superstructure example that coordinates existing mechanisms by named targets.
 */
public final class SuperstructureExample {
    /**
     * Example robot superstructure declaration.
     */
    public static final SuperstructureConfig CONFIG = Superstructures.create("robot")
            .part("intake", IntakeExample.CONFIG)
            .part("shooter", ShooterExample.CONFIG)
            // Superstructure states reference child mechanism state names.
            .state("idle", state -> state
                    .part("intake", "stopped")
                    .part("shooter", "idle"))
            .state("score", state -> state
                    .part("intake", "feed")
                    .part("shooter", "speaker"));

    private SuperstructureExample() {
    }
}
