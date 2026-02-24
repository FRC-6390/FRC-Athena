package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanisms.examples.MechanismInfrastructureExamples;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class MechanismInfrastructureExamplesTest {

    @Test
    void configuredMechanismAppliesSafeSysIdAndNetworkTablesSettings() {
        Mechanism mechanism = MechanismInfrastructureExamples.createConfiguredMechanism("intake");

        assertEquals(0.35, mechanism.sysId().rampRateVoltsPerSecond(), 1e-9);
        assertEquals(2.0, mechanism.sysId().stepVoltage(), 1e-9);
        assertEquals(5.0, mechanism.sysId().timeoutSeconds(), 1e-9);
        assertEquals(4.0, mechanism.sysId().voltageLimit(), 1e-9);
        assertEquals(0.05, mechanism.networkTables().periodSeconds(), 1e-9);
        assertEquals("Robot/Mechanisms/intake", mechanism.networkTables().ownerPath());
    }

    @Test
    void disableForTestModeShutsOffHooksAndControlLoops() {
        Mechanism mechanism = MechanismConfig.generic("test-mode").build();
        assertTrue(mechanism.hooksEnabled());
        assertTrue(mechanism.controlLoopsEnabled());

        MechanismInfrastructureExamples.disableForTestMode(mechanism);

        assertFalse(mechanism.hooksEnabled());
        assertFalse(mechanism.controlLoopsEnabled());
    }

    @Test
    void lazyFactoryBuildsOnlyOnceAndFlattens() {
        AtomicInteger buildCount = new AtomicInteger();
        RegisterableMechanismFactory factory =
                MechanismInfrastructureExamples.lazyFactory(MechanismConfig.generic("pivot"), buildCount);

        RegisterableMechanism first = factory.build();
        RegisterableMechanism second = factory.build();

        assertSame(first, second);
        assertEquals(1, buildCount.get());
        assertEquals(1, factory.flattenForRegistration().size());
    }

    @Test
    void relayConfigSanitizationClampsToSafeRanges() {
        MechanismPidAutotuners.RelayPositionConfig percentConfig =
                MechanismInfrastructureExamples.sanitizedRelayConfig(OutputType.PERCENT, 1.2, -0.5, 100.0, 100.0);
        assertEquals(0.75, percentConfig.relayOutput(), 1e-9);
        assertEquals(1e-6, percentConfig.errorBand(), 1e-12);
        assertEquals(30.0, percentConfig.timeoutSec(), 1e-9);
        assertEquals(12.0, percentConfig.targetCycles(), 1e-9);

        MechanismPidAutotuners.RelayPositionConfig voltageConfig =
                MechanismInfrastructureExamples.sanitizedRelayConfig(OutputType.VOLTAGE, 8.0, 0.02, 2.0, 2.0);
        assertEquals(6.0, voltageConfig.relayOutput(), 1e-9);
        assertEquals(4.0, voltageConfig.timeoutSec(), 1e-9);
        assertEquals(4.0, voltageConfig.targetCycles(), 1e-9);
    }

    @Test
    void vectorVisualizationBuildsExpectedNodeAndConversionsRoundTrip() {
        MechanismVisualizationConfig config =
                MechanismInfrastructureExamples.createVectorLineVisualization("TurretRoot");

        assertEquals("TurretRoot", config.rootName());
        assertTrue(config.nodes().stream().anyMatch(node -> node.name().equals("TurretRootVectorLine")));

        double volts = MechanismInfrastructureExamples.toVoltage(0.5, 12.0);
        double percent = MechanismInfrastructureExamples.toPercent(volts, 12.0);
        assertEquals(6.0, volts, 1e-9);
        assertEquals(0.5, percent, 1e-9);
    }

    @Test
    void defaultVisualizationsRenderAcrossMechanismTypes() {
        Map<String, Mechanism> mechanisms = Map.of(
                "generic", MechanismConfig.generic("generic").build(),
                "arm", MechanismConfig.arm("arm").build(),
                "elevator", MechanismConfig.elevator("elevator").build(),
                "flywheel", MechanismConfig.flywheel("flywheel").build(),
                "turret", MechanismConfig.turret("turret").build());

        for (Map.Entry<String, Mechanism> entry : mechanisms.entrySet()) {
            Mechanism mechanism = entry.getValue();
            assertNotNull(mechanism.visualization().mechanism2d(), "missing 2d visualization for " + entry.getKey());
            assertNotNull(
                    mechanism.visualization().mechanism3dPoses(),
                    "missing 3d pose map for " + entry.getKey());
        }
    }
}
