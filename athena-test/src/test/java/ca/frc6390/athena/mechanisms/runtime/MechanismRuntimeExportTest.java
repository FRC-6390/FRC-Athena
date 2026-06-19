package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.mechanisms.config.MechanismConfigExport;

final class MechanismRuntimeExportTest {
    @Test
    void snapshotAndExportWorkForV2BuiltMechanism() {
        Mechanism mechanism = MechanismDefinitions.build(
                Mechanisms.create("exportable")
                        .behavior(behavior -> behavior.control(control -> control
                                .pid("hold", pid -> pid
                                        .output(OutputType.VOLTAGE)
                                        .manual()
                                        .kp(0.3)
                                        .ki(0.0)
                                        .kd(0.0))))
                        .definition());

        MechanismConfigRecord snapshot = MechanismConfigIO.snapshot(mechanism);
        assertNotNull(snapshot);
        assertEquals(0, snapshot.motors().size());
        assertEquals(0, snapshot.encoders().size());

        var exported = MechanismConfigExport.export(mechanism);
        assertNotNull(exported);
        assertEquals("exportable", exported.name());
        assertNotNull(exported.control());
        assertNotNull(exported.control().pidProfiles());
        assertEquals(1, exported.control().pidProfiles().size());
        assertEquals("hold", exported.control().pidProfiles().getFirst().name());
        assertTrue(exported.encoders() == null || exported.encoders().isEmpty());
    }
}
