package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.runtime.MechanismRuntimeLowerer;

final class MechanismBangBangControlTest {

    @Test
    void calculateBangBangRawUsesHighLowAndTolerance() {
        MechanismRuntimeConfig.BangBangProfile profile =
                new MechanismRuntimeConfig.BangBangProfile(OutputType.PERCENT, 0.7, -0.4, 0.1, null, null);

        assertEquals(0.7, Mechanism.calculateBangBangRaw(profile, 0.0, 1.0), 1e-9);
        assertEquals(-0.4, Mechanism.calculateBangBangRaw(profile, 2.0, 1.0), 1e-9);
        assertEquals(0.0, Mechanism.calculateBangBangRaw(profile, 0.95, 1.0), 1e-9);
        assertEquals(0.0, Mechanism.calculateBangBangRaw(profile, 1.05, 1.0), 1e-9);
    }

    @Test
    void toleranceAndSanitizersHandleInvalidValues() {
        MechanismRuntimeConfig.BangBangProfile invalid =
                new MechanismRuntimeConfig.BangBangProfile(OutputType.PERCENT, Double.NaN, Double.NaN, Double.NaN, null, null);

        assertEquals(0.0, Mechanism.sanitizeBangBangLevel(Double.NaN), 1e-9);
        assertEquals(0.0, Mechanism.sanitizeBangBangTolerance(Double.NaN), 1e-9);
        assertEquals(0.0, Mechanism.calculateBangBangRaw(invalid, 0.0, 1.0), 1e-9);
        assertFalse(Mechanism.isBangBangWithinTolerance(invalid, 0.0, 1.0));
    }

    @Test
    void controlSectionRegistersAndResolvesBangBangProfiles() {
        var runtime = MechanismRuntimeLowerer.lower(
                Mechanisms.create("bangbang")
                        .behavior(behavior -> behavior.control(control -> control
                                .bangBang("assist", bangBang -> bangBang
                                        .output(OutputType.VOLTAGE)
                                        .high(5.0)
                                        .low(-3.0)
                                        .tolerance(0.2))))
                        .definition());

        MechanismRuntimeConfig.BangBangProfile resolved = runtime.controlLoopBangBangProfiles().get("assist");
        assertEquals(OutputType.VOLTAGE, resolved.outputType());
        assertEquals(5.0, resolved.highOutput(), 1e-9);
        assertEquals(-3.0, resolved.lowOutput(), 1e-9);
        assertEquals(0.2, resolved.tolerance(), 1e-9);
        assertEquals(1, runtime.controlLoops().size());
        assertEquals("assist", runtime.controlLoops().get(0).name());
        assertTrue(runtime.controlLoopBangBangProfiles().containsKey("assist"));
    }
}
