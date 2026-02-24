package ca.frc6390.athena.core.auto;

import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.RobotAuto;
import org.junit.jupiter.api.Test;

final class AutoBackendsContractTest {

    @Test
    void coreOnlyClasspathHasNoAutoBackends() {
        assertTrue(AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER).isEmpty());
        assertTrue(AutoBackends.forSource(RobotAuto.AutoSource.CHOREO).isEmpty());
        assertTrue(AutoBackends.forSource(RobotAuto.AutoSource.CUSTOM).isEmpty());
    }
}
