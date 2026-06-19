package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import org.junit.jupiter.api.Test;

final class MechanismDisabledBehaviorTest {
    private enum State implements SetpointProvider<Double> {
        Off(0.0);

        private final double setpoint;

        State(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    @Test
    void disabledDefinitionStillBuildsStatefulMechanism() {
        @SuppressWarnings("unchecked")
        StatefulMechanism<State> mechanism = (StatefulMechanism<State>) MechanismDefinitions.build(
                Mechanisms.stateful("arm", State.Off)
                        .disabled(true)
                        .definition());

        assertNotNull(mechanism);
        assertTrue(mechanism.configDisabled());
        assertEquals("arm", mechanism.getName());
    }

    @Test
    void disabledMechanismIgnoresControlMutations() {
        Mechanism mechanism = MechanismDefinitions.build(
                Mechanisms.create("inert")
                        .disabled(true)
                        .definition());

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
    void disabledMechanismSkipsStateAutomationAndControlLoops() {
        AtomicInteger hookRuns = new AtomicInteger();
        AtomicInteger loopRuns = new AtomicInteger();

        @SuppressWarnings("unchecked")
        StatefulMechanism<State> mechanism = (StatefulMechanism<State>) MechanismDefinitions.build(
                Mechanisms.stateful("disabled-runtime", State.Off)
                        .disabled(true)
                        .behavior(behavior -> behavior
                                .automation(automation -> automation.onStatePeriodic(
                                        ctx -> hookRuns.incrementAndGet(),
                                        State.Off))
                                .control(control -> control.customLoop("loop", loop -> loop.custom(ctx -> {
                                    loopRuns.incrementAndGet();
                                    return 1.0;
                                }))))
                        .definition());

        mechanism.periodic();
        mechanism.periodic();
        mechanism.update();

        assertEquals(0, hookRuns.get());
        assertEquals(0, loopRuns.get());
        assertEquals(0.0, mechanism.output(), 1e-9);
    }
}
