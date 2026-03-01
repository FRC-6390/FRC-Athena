package ca.frc6390.athena.core.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.RobotAuto;
import org.junit.jupiter.api.Test;

final class AutoBackendsContractTest {

    @Test
    void backendResolutionIsDeterministicForActiveClasspath() {
        boolean pathPlannerPresent = AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).isPresent();
        boolean choreoPresent = AutoBackends.forSource(RobotAuto.AutoSource.CHOREO).isPresent();

        assertEquals(pathPlannerPresent, AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).isPresent());
        assertEquals(choreoPresent, AutoBackends.forSource(RobotAuto.AutoSource.CHOREO).isPresent());
        assertTrue(AutoBackends.forSource(RobotAuto.AutoSource.CUSTOM).isEmpty());
    }
}
