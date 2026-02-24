package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import org.junit.jupiter.api.Test;

final class MechanismConfigTurretAutoContinuousPidTest {

    @Test
    void enablesContinuousPidWhenUnboundedAndConversionLooksLikeDegrees() {
        MechanismConfigRecord base = MechanismConfigRecord.defaults();
        MechanismConfigRecord cfg = base.toBuilder()
                .pidController(new PIDController(0.0, 0.0, 0.0))
                .encoderConversion(360.0)
                .minBound(Double.NaN)
                .maxBound(Double.NaN)
                .build();

        MechanismConfigRecord out = MechanismConfig.applyAutoContinuousPidForUnboundedTurret(cfg);

        assertTrue(out.pidContinous());
        assertEquals(-180.0, out.continousMin(), 1e-9);
        assertEquals(180.0, out.continousMax(), 1e-9);
    }

    @Test
    void doesNotEnableContinuousPidWhenBoundsAreSet() {
        MechanismConfigRecord base = MechanismConfigRecord.defaults();
        MechanismConfigRecord cfg = base.toBuilder()
                .pidController(new PIDController(0.0, 0.0, 0.0))
                .encoderConversion(360.0)
                .minBound(0.0)
                .maxBound(270.0)
                .build();

        MechanismConfigRecord out = MechanismConfig.applyAutoContinuousPidForUnboundedTurret(cfg);

        assertFalse(out.pidContinous());
    }

    @Test
    void doesNotEnableContinuousPidWhenConversionIsInvalid() {
        MechanismConfigRecord base = MechanismConfigRecord.defaults();
        MechanismConfigRecord cfg = base.toBuilder()
                .pidController(new PIDController(0.0, 0.0, 0.0))
                .encoderConversion(Double.NaN)
                .build();

        MechanismConfigRecord out = MechanismConfig.applyAutoContinuousPidForUnboundedTurret(cfg);

        assertFalse(out.pidContinous());
    }
}

