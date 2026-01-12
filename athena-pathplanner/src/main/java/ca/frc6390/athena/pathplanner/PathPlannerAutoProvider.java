package ca.frc6390.athena.pathplanner;

import ca.frc6390.athena.core.AutoRegistry;
import ca.frc6390.athena.core.RobotAuto;

/**
 * Registers PathPlanner auto engine with the central registry.
 */
public class PathPlannerAutoProvider implements AutoRegistry.Provider {
    @Override
    public void register(AutoRegistry registry) {
        registry.add("auto:pathplanner", RobotAuto.AutoSource.PATH_PLANNER);
    }
}
