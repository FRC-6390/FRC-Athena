package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class MechanismConfigDisabledBehaviorTest {

    @Test
    void disabledConfigStillBuildsTypedMechanism() {
        MechanismConfig<ArmMechanism> cfg = MechanismConfig.arm("arm").disabled(true);

        ArmMechanism mechanism = cfg.build();

        assertNotNull(mechanism);
        assertTrue(mechanism.configDisabled());
        assertEquals("arm", mechanism.getName());
    }

    @Test
    void disabledMechanismIgnoresControlMutations() {
        Mechanism mechanism = MechanismConfig.generic("inert").disabled(true).build();

        mechanism.control().setpoint(5.0);
        mechanism.control().nudge(1.0);
        mechanism.control().manualOverride(true);
        mechanism.control().output(0.8);
        mechanism.control().pidEnabled(true);
        mechanism.control().feedforwardEnabled(true);
        mechanism.update();

        assertTrue(mechanism.configDisabled());
        assertEquals(0.0, mechanism.setpoint(), 1e-9);
        assertEquals(0.0, mechanism.nudge(), 1e-9);
        assertEquals(0.0, mechanism.output(), 1e-9);
        assertFalse(mechanism.manualOverride());
        assertFalse(mechanism.pidEnabled());
        assertFalse(mechanism.feedforwardEnabled());
        assertFalse(mechanism.hooksEnabled());
        assertFalse(mechanism.controlLoopsEnabled());
        assertTrue(mechanism.atSetpoint());
    }

    @Test
    void disabledMechanismSkipsHooksAndControlLoops() {
        AtomicInteger hookRuns = new AtomicInteger();
        AtomicInteger loopRuns = new AtomicInteger();

        MechanismConfig<Mechanism> cfg = MechanismConfig.generic("disabled-runtime")
                .disabled(true)
                .hooks(h -> h.onRobotPeriodic(mech -> hookRuns.incrementAndGet()))
                .control(c -> c.controlLoop("loop", 20.0, ctx -> {
                    loopRuns.incrementAndGet();
                    return 1.0;
                }));

        Mechanism mechanism = cfg.build();
        mechanism.periodic();
        mechanism.periodic();
        mechanism.update();

        assertEquals(0, hookRuns.get());
        assertEquals(0, loopRuns.get());
        assertEquals(0.0, mechanism.output(), 1e-9);
    }
}
