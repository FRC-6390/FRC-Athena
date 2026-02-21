package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

final class MechanismBangBangControlTest {

    @Test
    void calculateBangBangRawUsesHighLowAndTolerance() {
        MechanismConfig.BangBangProfile profile =
                new MechanismConfig.BangBangProfile(OutputType.PERCENT, 0.7, -0.4, 0.1);

        assertEquals(0.7, Mechanism.calculateBangBangRaw(profile, 0.0, 1.0), 1e-9);
        assertEquals(-0.4, Mechanism.calculateBangBangRaw(profile, 2.0, 1.0), 1e-9);
        assertEquals(0.0, Mechanism.calculateBangBangRaw(profile, 0.95, 1.0), 1e-9);
        assertEquals(0.0, Mechanism.calculateBangBangRaw(profile, 1.05, 1.0), 1e-9);
    }

    @Test
    void toleranceAndSanitizersHandleInvalidValues() {
        MechanismConfig.BangBangProfile invalid =
                new MechanismConfig.BangBangProfile(OutputType.PERCENT, Double.NaN, Double.NaN, Double.NaN);

        assertEquals(0.0, Mechanism.sanitizeBangBangLevel(Double.NaN), 1e-9);
        assertEquals(0.0, Mechanism.sanitizeBangBangTolerance(Double.NaN), 1e-9);
        assertEquals(0.0, Mechanism.calculateBangBangRaw(invalid, 0.0, 1.0), 1e-9);
        assertFalse(Mechanism.isBangBangWithinTolerance(invalid, 0.0, 1.0));
    }

    @Test
    void controlSectionRegistersAndResolvesBangBangProfiles() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c
                .bangBang("assist", OutputType.VOLTAGE, 5.0, -3.0, 0.2)
                .periodic("assist"));

        MechanismConfig.BangBangProfile resolved = cfg.controlLoopBangBangProfiles().get("assist");
        assertEquals(OutputType.VOLTAGE, resolved.outputType());
        assertEquals(5.0, resolved.highOutput(), 1e-9);
        assertEquals(-3.0, resolved.lowOutput(), 1e-9);
        assertEquals(0.2, resolved.tolerance(), 1e-9);
        assertEquals(1, cfg.controlLoops().size());
        assertEquals("assist", cfg.controlLoops().get(0).name());
        assertTrue(cfg.controlLoopBangBangProfiles().containsKey("assist"));
    }
}
