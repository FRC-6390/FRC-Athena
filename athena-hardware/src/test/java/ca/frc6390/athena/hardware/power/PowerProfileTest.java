package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class PowerProfileTest {
    @Test
    void builderStoresTypedValues() {
        PowerProfile profile = PowerProfile.of()
                .sustained(Units.Amps.of(60))
                .burst(Units.Amps.of(90), Units.Seconds.of(0.5))
                .rampLimit(Units.Amps.of(10), Units.Seconds.of(0.1))
                .share(3.0)
                .onSag(Sag.shed(0.5).below(Units.Volts.of(9)));
        assertEquals(60.0, profile.sustainedCurrent().in(edu.wpi.first.units.Units.Amps), 1e-9);
        assertEquals(90.0, profile.burstCurrent().in(edu.wpi.first.units.Units.Amps), 1e-9);
        assertEquals(0.5, profile.burstDuration().in(edu.wpi.first.units.Units.Seconds), 1e-9);
        assertEquals(100.0, profile.rampAmpsPerSecond(), 1e-9);
        assertEquals(3.0, profile.share(), 1e-9);
        assertEquals(0.5, profile.sag().shedFraction(), 1e-9);
        assertEquals(9.0, profile.sag().threshold().in(edu.wpi.first.units.Units.Volts), 1e-9);
    }

    @Test
    void rejectsInvalidValues() {
        assertThrows(IllegalArgumentException.class, () -> PowerProfile.of().sustained(Units.Amps.of(-1)));
        assertThrows(IllegalArgumentException.class, () -> PowerProfile.of().share(0));
        assertThrows(IllegalArgumentException.class,
                () -> PowerProfile.of().rampLimit(Units.Amps.of(0), Units.Seconds.of(1)));
        assertThrows(IllegalArgumentException.class, () -> Sag.shed(0.0));
        assertThrows(IllegalArgumentException.class, () -> Sag.shed(0.5).below(Units.Volts.of(0)));
    }

    @Test
    void copyIsDetached() {
        PowerProfile original = PowerProfile.of().share(2.0);
        PowerProfile copy = original.copy();
        original.share(5.0);
        assertEquals(2.0, copy.share(), 1e-9);
        assertNull(copy.sag());
        assertNotNull(original);
        assertNotSame(original, copy);
    }

    @Test
    void copyRetainsElectricalSettingsButHasIndependentStartupMembership() {
        PowerProfile original = PowerProfile.of().sustained(Units.Amps.of(40))
                .burst(Units.Amps.of(60), Units.Seconds.of(0.5))
                .rampLimit(Units.Amps.of(10), Units.Seconds.of(0.1))
                .share(3.0).onSag(Sag.shed(0.5).below(Units.Volts.of(8)))
                .startTogether("swerve");
        PowerProfile copy = original.copy();
        assertNotSame(original, copy);
        assertEquals(40.0, copy.sustainedAmps());
        assertEquals(60.0, copy.burstAmps());
        assertEquals(0.5, copy.burstSeconds());
        assertEquals(100.0, copy.rampAmpsPerSecond());
        assertEquals(3.0, copy.share());
        assertEquals(original.sag(), copy.sag());
        assertNull(copy.startupGroup());
        assertEquals("swerve", original.startupGroup());
        copy.sustained(Units.Amps.of(20)).startTogether("intake");
        assertEquals(40.0, original.sustainedAmps());
        assertEquals("swerve", original.startupGroup());
    }

    @Test
    void startupKeyCanBeRemovedButCannotBeBlank() {
        PowerProfile profile = PowerProfile.of().startTogether("drive");
        assertThrows(IllegalArgumentException.class, () -> profile.startTogether("  "));
        profile.startTogether(null);
        assertNull(profile.startupGroup());
    }

    @Test
    void policyDefaultsAndOverrides() {
        PowerPolicy defaults = PowerPolicy.defaults();
        assertEquals(7.0, defaults.floorVolts(), 1e-9);
        assertEquals(0.0, defaults.staggerSeconds(), 1e-9);
        assertEquals(true, defaults.estimateBatteryResistance());
        PowerPolicy tuned = defaults.floor(Units.Volts.of(7.5)).staggerStarts(Units.Seconds.of(0.2));
        assertEquals(7.5, tuned.floorVolts(), 1e-9);
        assertEquals(0.2, tuned.staggerSeconds(), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> defaults.floor(Units.Volts.of(-1)));
    }
}
