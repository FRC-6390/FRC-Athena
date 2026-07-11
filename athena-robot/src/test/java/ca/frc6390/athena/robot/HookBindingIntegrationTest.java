package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.sim.hardware.SimMotorHandle.CommandKind;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

class HookBindingIntegrationTest {
    @Test
    void heldActionOnRisingPulseReleasesOnFollowingTick() {
        SimulationSession simulation = SimulationSession.create();
        DigitalInputDevice input = DigitalInputDevice.rio(1);
        EdgeActionMechanism mechanism = new EdgeActionMechanism(input);
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
        simulation.digitalInput(input).raw(false);
        runtime.teleopPeriodic(0.0, 0.02);

        simulation.digitalInput(input).raw(true);
        runtime.teleopPeriodic(0.02, 0.02);
        assertEquals(CommandKind.PERCENT, simulation.motor(mechanism.motor).commandKind());
        assertEquals(0.5, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);

        runtime.teleopPeriodic(0.04, 0.02);
        assertEquals(CommandKind.NEUTRAL, simulation.motor(mechanism.motor).commandKind());
    }

    @Test
    void transientLatchedEdgeReachesHooksInEveryMechanism() {
        SimulationSession simulation = SimulationSession.create();
        DigitalInputDevice input = DigitalInputDevice.rio(2);
        CountingMechanism first = new CountingMechanism(input, true);
        CountingMechanism second = new CountingMechanism(input, true);
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(new SharedRoot(first, second));
        simulation.digitalInput(input).raw(false);
        runtime.teleopPeriodic(0.0, 0.02);

        simulation.digitalInput(input).raw(true);
        simulation.withDigitalInputs(input::sample);
        simulation.digitalInput(input).raw(false);
        simulation.withDigitalInputs(input::sample);
        runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(1, first.activations);
        assertEquals(1, second.activations);
    }

    @Test
    void invertedInputMapsRawPressAndReleaseToLogicalEdges() {
        SimulationSession simulation = SimulationSession.create();
        DigitalInputDevice input = DigitalInputDevice.rio(3).inverted();
        InvertedEdgeMechanism mechanism = new InvertedEdgeMechanism(input);
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
        simulation.digitalInput(input).raw(true);
        runtime.teleopPeriodic(0.0, 0.02);

        simulation.digitalInput(input).raw(false);
        runtime.teleopPeriodic(0.02, 0.02);
        assertEquals(1, mechanism.presses);
        assertEquals(0, mechanism.releases);

        simulation.digitalInput(input).raw(true);
        runtime.teleopPeriodic(0.04, 0.02);
        assertEquals(1, mechanism.presses);
        assertEquals(1, mechanism.releases);
    }

    private static final class EdgeActionMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 91);
        private final DigitalInputDevice input;
        private final HookBinding hook;

        private EdgeActionMechanism(DigitalInputDevice input) {
            this.input = input;
            hook = Events.when(input).rising().whileActive(motor.percent(0.5));
        }
    }

    private static final class CountingMechanism implements Mechanism {
        private final DigitalInputDevice input;
        private final HookBinding hook;
        private int activations;

        private CountingMechanism(DigitalInputDevice input, boolean rising) {
            this.input = input;
            hook = (rising ? Events.when(input).rising() : Events.when(input).falling())
                    .onStart(() -> activations++);
        }
    }

    private record SharedRoot(CountingMechanism first, CountingMechanism second) implements Mechanism {
    }

    private static final class InvertedEdgeMechanism implements Mechanism {
        private final DigitalInputDevice input;
        private final HookBinding press;
        private final HookBinding release;
        private int presses;
        private int releases;

        private InvertedEdgeMechanism(DigitalInputDevice input) {
            this.input = input;
            press = Events.when(input).rising().onStart(() -> presses++);
            release = Events.when(input).falling().onStart(() -> releases++);
        }
    }
}
