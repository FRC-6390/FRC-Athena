package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

final class MechanismPidControlTest {

    @Test
    void controlSectionRegistersProfiledPidConstraints() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c
                .pid("profiled", p -> p
                        .kp(0.2)
                        .ki(0.0)
                        .kd(0.0)
                        .constraints(35.0, 120.0))
                .periodic("profiled"));

        MechanismConfig.PidProfile resolved = cfg.controlLoopPidProfiles.get("profiled");
        assertEquals(35.0, resolved.maxVelocity(), 1e-9);
        assertEquals(120.0, resolved.maxAcceleration(), 1e-9);
        assertEquals(1, cfg.controlLoops.size());
        assertEquals("profiled", cfg.controlLoops.get(0).name());
    }

    @Test
    void pidConstraintsRequireBothVelocityAndAcceleration() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        assertThrows(
                IllegalArgumentException.class,
                () -> cfg.control(c -> c.pid("bad", p -> p.constraints(25.0, Double.NaN))));
    }
}
