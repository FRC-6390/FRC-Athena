package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.runtime.MechanismRuntimeLowerer;
import org.junit.jupiter.api.Test;

final class MechanismConfigTurretAutoContinuousPidTest {

    @Test
    void enablesContinuousPidWhenContinuousAngularIdentityUsesDegrees() {
        MechanismConfigRecord out = MechanismRuntimeLowerer.lower(
                Mechanisms.create("turret")
                        .identity(identity -> identity
                                .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)
                                .continuousRotation())
                        .definition())
                .data();

        assertTrue(out.pidContinous());
        assertEquals(-180.0, out.continousMin(), 1e-9);
        assertEquals(180.0, out.continousMax(), 1e-9);
    }

    @Test
    void doesNotEnableContinuousPidWhenMechanismIsBounded() {
        MechanismConfigRecord out = MechanismRuntimeLowerer.lower(
                Mechanisms.create("turret")
                        .identity(identity -> identity
                                .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)
                                .travelRange(0.0, 270.0))
                        .definition())
                .data();

        assertFalse(out.pidContinous());
    }

    @Test
    void doesNotEnableContinuousPidWhenContinuousUnitsAreUnsupported() {
        MechanismConfigRecord out = MechanismRuntimeLowerer.lower(
                Mechanisms.create("elevator")
                        .identity(identity -> identity
                                .positionDomain(PositionDomainKind.LINEAR, PositionUnit.METERS)
                                .continuousRotation())
                        .definition())
                .data();

        assertFalse(out.pidContinous());
    }
}
