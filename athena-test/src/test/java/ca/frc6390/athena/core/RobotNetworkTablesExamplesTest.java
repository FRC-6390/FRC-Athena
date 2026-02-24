package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.RobotNetworkTablesExamples;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

final class RobotNetworkTablesExamplesTest {

    @Test
    void runtimeConfigExampleAppliesFlagAndPublishSettings() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startLocal();
        try {
            RobotNetworkTables nt = new RobotNetworkTables(instance);
            RobotNetworkTablesExamples.configureFastLocalPublishing(nt);

            assertTrue(nt.isPublishingEnabled());
            assertTrue(nt.enabled(RobotNetworkTables.Flag.DRIVETRAIN_SPEED_WIDGETS));
            assertEquals(0.05, nt.getDefaultPeriodSeconds(), 1e-9);
        } finally {
            instance.close();
        }
    }

    @Test
    void mechanismToggleExampleEnablesAllTelemetryGroups() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startLocal();
        try {
            RobotNetworkTables nt = new RobotNetworkTables(instance);
            RobotNetworkTables.MechanismToggles toggles =
                    RobotNetworkTablesExamples.enableMechanismTelemetry(nt, "Mechanisms/Test");

            assertTrue(toggles.detailsEnabled());
            assertTrue(toggles.motorsEnabled());
            assertTrue(toggles.encoderEnabled());
            assertTrue(toggles.constraintsEnabled());
            assertTrue(toggles.sensorsEnabled());
            assertTrue(toggles.controlEnabled());
            assertTrue(toggles.inputsEnabled());
            assertTrue(toggles.simulationEnabled());
            assertTrue(toggles.sysIdEnabled());
        } finally {
            instance.close();
        }
    }
}
