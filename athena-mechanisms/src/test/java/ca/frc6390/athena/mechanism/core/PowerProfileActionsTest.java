package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import org.junit.jupiter.api.Test;

class PowerProfileActionsTest {
    @Test
    void laterAttachmentDoesNotReplaceSharedBaselineAndCopyRestoresIndependently() {
        PowerProfile shared = PowerProfile.of().share(2.0).startTogether("declared");
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).power(shared);
        shared.share(3.0).startTogether("tuned");
        PowerProfile copied = shared.copy();
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).power(shared);
        MotorDevice independent = MotorDevice.of(MotorKinds.KRAKEN_X60, 3).power(copied);
        copied.share(5.0).startTogether("independent");

        run(shared.<Action>restore());

        assertSame(shared, first.power());
        assertSame(shared, second.power());
        assertEquals(2.0, first.power().share());
        assertEquals(2.0, second.power().share());
        assertEquals("declared", shared.startupGroup());
        assertEquals(5.0, independent.power().share());

        run(copied.<Action>restore());

        assertEquals(3.0, independent.power().share());
        assertNull(copied.startupGroup());
        assertEquals(2.0, shared.share());
        assertEquals("declared", shared.startupGroup());
    }

    @Test
    void updateBeforeAttachmentCapturesSettingsWithoutChangingSharedIdentity() {
        PowerProfile profile = PowerProfile.of().share(2.0).startTogether("declared");
        run(profile.<Action>update(value -> value.share(7.0).startTogether("temporary")));
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 4).power(profile);
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 5).power(profile);
        assertEquals(7.0, first.power().share());

        run(profile.<Action>restore());

        assertSame(profile, first.power());
        assertSame(profile, second.power());
        assertEquals(2.0, profile.share());
        assertEquals("declared", profile.startupGroup());
    }

    private static void run(Action action) {
        Harness mechanism = new Harness(action);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, new ActionContext() {});
        runtime.set(action);
        runtime.periodic(new MechanismContext(0.0, 0.0, 0.02, true, false, false), EventContext.empty());
    }

    private static final class Harness implements Mechanism {
        final Action operation;
        Harness(Action operation) { this.operation = operation; }
    }
}
