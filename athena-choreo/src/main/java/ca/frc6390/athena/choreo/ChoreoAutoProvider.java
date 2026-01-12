package ca.frc6390.athena.choreo;

import ca.frc6390.athena.core.AutoRegistry;
import ca.frc6390.athena.core.RobotAuto;

/**
 * Registers Choreo auto engine with the central registry.
 */
public class ChoreoAutoProvider implements AutoRegistry.Provider {
    @Override
    public void register(AutoRegistry registry) {
        registry.add("auto:choreo", RobotAuto.AutoSource.CHOREO);
    }
}
