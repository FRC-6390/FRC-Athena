package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardModel;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import org.junit.jupiter.api.Test;

final class MechanismControlLoopDisableTest {
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
    void pidDisableStopsNamedPidLoopOutputImmediately() {
        Mechanism mechanism = MechanismDefinitions.build(
                Mechanisms.create("pid-test")
                        .behavior(behavior -> behavior.control(control -> control
                                .pid("hold", pid -> pid.kp(1.0).ki(0.0).kd(0.0))))
                        .definition());

        mechanism.control().setpoint(2.0);
        mechanism.update();
        assertTrue(Math.abs(mechanism.output()) > 1e-9);

        mechanism.control().pidEnabled(false);
        mechanism.update();
        assertEquals(0.0, mechanism.output(), 1e-9);
    }

    @Test
    void feedforwardDisableStopsNamedFeedforwardLoopOutputImmediately() {
        Mechanism mechanism = MechanismDefinitions.build(
                Mechanisms.create("ff-test")
                        .identity(identity -> identity.positionDomain(
                                ca.frc6390.athena.api.mechanism.identity.PositionDomainKind.VELOCITY,
                                ca.frc6390.athena.api.mechanism.identity.PositionUnit.RADIANS))
                        .behavior(behavior -> behavior.control(control -> control
                                .feedforward("vel", ff -> ff
                                        .simple()
                                        .ks(0.1)
                                        .kv(0.5)
                                        .ka(0.0)
                                        .setpointSource(MechanismSetpointSource.Setpoint))))
                        .definition());

        mechanism.control().setpoint(4.0);
        mechanism.update();
        assertTrue(Math.abs(mechanism.output()) > 1e-9);

        mechanism.control().feedforwardEnabled(false);
        mechanism.update();
        assertEquals(0.0, mechanism.output(), 1e-9);
    }

    @Test
    void disableAllControlLoopsTurnsOffCustomAndNamedLoops() {
        Mechanism mechanism = MechanismDefinitions.build(
                Mechanisms.create("custom")
                        .behavior(behavior -> behavior.control(control -> control
                                .customLoop("custom", loop -> loop.custom(ctx -> 1.25))))
                        .definition());

        mechanism.update();
        assertEquals(1.25, mechanism.output(), 1e-9);

        mechanism.disableAllControlLoops();
        mechanism.update();

        assertEquals(0.0, mechanism.output(), 1e-9);
        assertFalse(mechanism.loops().enabled("custom"));
    }

    @Test
    void disableAllHooksStopsStatePeriodicAutomationExecution() {
        AtomicInteger runs = new AtomicInteger();

        @SuppressWarnings("unchecked")
        StatefulMechanism<State> mechanism = (StatefulMechanism<State>) MechanismDefinitions.build(
                Mechanisms.stateful("hooks", State.Off)
                        .behavior(behavior -> behavior.automation(automation -> automation.onStatePeriodic(
                                ctx -> runs.incrementAndGet(),
                                State.Off)))
                        .definition());

        mechanism.periodic();
        assertEquals(1, runs.get());

        mechanism.disableAllHooks();
        mechanism.periodic();
        assertEquals(1, runs.get());
    }

    @Test
    void feedforwardBuilderSupportsTypedProfiles() {
        var definition = Mechanisms.create("typed-ff")
                .behavior(behavior -> behavior.control(control -> control
                        .feedforward("ff", ff -> ff
                                .arm()
                                .ks(0.2)
                                .kg(0.4)
                                .kv(0.6)
                                .ka(0.8)
                                .tolerance(0.05))))
                .definition();

        var controller = (MechanismFeedforwardControllerDefinition) definition.loops().getFirst().controller();
        assertEquals(MechanismFeedforwardModel.ARM, controller.model());
        assertEquals(0.2, controller.kS(), 1e-9);
        assertEquals(0.4, controller.kG(), 1e-9);
        assertEquals(0.6, controller.kV(), 1e-9);
        assertEquals(0.8, controller.kA(), 1e-9);
        assertEquals(0.05, controller.tolerance().orElseThrow(), 1e-9);
    }
}
