package ca.frc6390.athena.mechanisms.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorRegistry;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.OutputType;
import org.junit.jupiter.api.Test;

final class MechanismConfigApplyExportTest {

    @Test
    void applyThenExportPreservesDataOnlyProfilesAndControlSettings() {
        MotorControllerType dummyMotor = () -> "dummy:motor";
        MotorRegistry.get().add(dummyMotor);

        MechanismConfig<Mechanism> cfg = MechanismConfig.generic("lift");
        MechanismConfigFile file = new MechanismConfigFile(
                "lift",
                "Mechanism",
                "rotations",
                new MechanismMotorsConfig(
                        "rio",
                        "brake",
                        40.0,
                        List.of(new MechanismMotorConfig(null, "dummy:motor", 5, false))),
                new MechanismEncoderConfig("custom", null, 5, true, false, 100.0, 1.0, 0.0, 0.0, null, null),
                new MechanismConstraintsConfig(-2.0, 8.0, null, new MechanismMotionLimitsConfig(5.0, 12.0)),
                new MechanismSensorsConfig(
                        List.of(new MechanismLimitSwitchConfig(0, false, 0.0, true, "PositiveInput", "top", 0.01)),
                        10.0),
                new MechanismControlConfig(
                        "voltage",
                        false,
                        true,
                        -180.0,
                        180.0,
                        0.05,
                        List.of(new MechanismPidConfig("hold", 1.0, 0.0, 0.1, null, null, 0.05, 25.0, 100.0, "velocity")),
                        null,
                        List.of(new MechanismFeedforwardConfig("main", "simple", 0.2, 0.0, 1.8, 0.05, 0.03, "input:ff_vel"))),
                new MechanismSimConfig(new MechanismSimSimpleMotorConfig(0.01, 12.0, 1.0), null, null));

        MechanismConfigApplier.apply(cfg, file);

        assertEquals(OutputType.VOLTAGE, cfg.data().outputType());
        assertEquals(-2.0, cfg.data().minBound(), 1e-9);
        assertEquals(8.0, cfg.data().maxBound(), 1e-9);
        assertEquals(5.0, cfg.data().motionLimits().maxVelocity(), 1e-9);
        assertEquals(12.0, cfg.data().motionLimits().maxAcceleration(), 1e-9);
        assertTrue(cfg.controlLoopPidProfiles().containsKey("hold"));
        assertTrue(cfg.controlLoopFeedforwardProfiles().containsKey("main"));
        assertEquals(25.0, cfg.controlLoopPidProfiles().get("hold").maxVelocity(), 1e-9);
        assertEquals(100.0, cfg.controlLoopPidProfiles().get("hold").maxAcceleration(), 1e-9);

        MechanismConfigFile exported = MechanismConfigExport.export(cfg, "Mechanism");
        assertEquals("lift", exported.name());
        assertNotNull(exported.control());
        assertEquals("VOLTAGE", exported.control().output());
        assertEquals(1, exported.control().pidProfiles().size());
        assertEquals("hold", exported.control().pidProfiles().get(0).name());
        assertEquals("velocity", exported.control().pidProfiles().get(0).source());
        assertEquals(25.0, exported.control().pidProfiles().get(0).maxVelocity());
        assertEquals(100.0, exported.control().pidProfiles().get(0).maxAcceleration());
        assertEquals(1, exported.control().ffProfiles().size());
        assertEquals("simple", exported.control().ffProfiles().get(0).type());
        assertEquals("input:ff_vel", exported.control().ffProfiles().get(0).source());

        String json = MechanismConfigExport.toJson(exported);
        assertTrue(json.contains("\"mechanism_type\""));

        String toml = MechanismConfigExport.toToml(exported);
        assertTrue(toml.contains("mechanism_type"));
    }
}
