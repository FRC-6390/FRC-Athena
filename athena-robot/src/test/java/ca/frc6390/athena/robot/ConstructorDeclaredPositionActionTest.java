package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import ca.frc6390.athena.sim.hardware.SimMotorHandle.CommandKind;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class ConstructorDeclaredPositionActionTest {
    @Test
    void constructorDeclaredPositionActionIsOwnedAndReevaluatesItsSupplier() {
        AtomicReference<Double> target = new AtomicReference<>(4.0);
        ConstructorActionMechanism mechanism = new ConstructorActionMechanism(target, false);
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        runtime.request(mechanism.trackTarget);
        runtime.teleopPeriodic(0.0, 0.02);

        assertEquals(CommandKind.POSITION, simulation.motor(mechanism.motor).commandKind());
        assertEquals(4.0, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);

        target.set(7.0);
        runtime.teleopPeriodic(0.02, 0.02);
        assertEquals(7.0, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);
    }

    @Test
    void constructorDeclaredPositionActionRunsThroughPlannerAndProfile() {
        AtomicReference<Double> target = new AtomicReference<>(90.0);
        ConstructorActionMechanism mechanism = new ConstructorActionMechanism(target, true);
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
        simulation.motor(mechanism.motor).state(0.0, 0.0);

        runtime.request(mechanism.trackTarget);
        runtime.teleopPeriodic(0.0, 0.02);
        runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(CommandKind.VOLTAGE, simulation.motor(mechanism.motor).commandKind());
        assertNotEquals(0.0, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);

        for (int cycle = 2; cycle <= 50; cycle++) {
            runtime.teleopPeriodic(cycle * 0.02, 0.02);
        }
        assertEquals(12.0, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);
    }

    @Test
    void constructorDeclaredTurretShapeUsesExternalEncoderAndCommandsVoltage() {
        AtomicReference<Double> target = new AtomicReference<>(90.0);
        ExternalEncoderMechanism mechanism = new ExternalEncoderMechanism(target);
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);
        simulation.encoder(mechanism.encoder)
                .positionRotations(mechanism.encoder.rotationsFromPosition(0.0));

        runtime.request(mechanism.trackTarget);
        runtime.teleopPeriodic(0.0, 0.02);
        runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(CommandKind.VOLTAGE, simulation.motor(mechanism.motor).commandKind());
        assertNotEquals(0.0, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);

        for (int cycle = 2; cycle <= 50; cycle++) {
            runtime.teleopPeriodic(cycle * 0.02, 0.02);
        }
        assertEquals(1.35, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);

        double firstVoltage = simulation.motor(mechanism.motor).commandValue();
        target.set(-45.0);
        runtime.teleopPeriodic(1.02, 0.02);
        assertNotEquals(firstVoltage, simulation.motor(mechanism.motor).commandValue(), 1.0e-9);
    }

    private static final class ConstructorActionMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 86);
        private final ControlBinding position;
        private final Action trackTarget;

        private ConstructorActionMechanism(AtomicReference<Double> target, boolean planned) {
            ControlBinding configured = Controls.position(motor)
                    .feedback(motor.encoder())
                    .pid(1.0, 0.0, 0.0);
            position = planned
                    ? configured
                            .planner(MotionPlanners.boundedAngular(360.0))
                            .profile(MotionProfiles.trapezoid(180.0, 360.0))
                    : configured;
            trackTarget = position.position(() -> target.get());
        }
    }

    private static final class ExternalEncoderMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 87);
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 88)
                .inverted()
                .gearRatio(1.0 / 7.0)
                .conversion(360.0);
        private final ControlBinding position = Controls.position(motor)
                .feedback(encoder)
                .pid(0.015, 0.0, 0.0)
                .constraint(Constraints.range(Range.degrees(-136.0, 204.0)))
                .constraint(Constraints.require(() -> true))
                .planner(MotionPlanners.boundedAngular(360.0))
                .profile(MotionProfiles.trapezoid(700.0, 1000.0));
        private final Action trackTarget;

        private ExternalEncoderMechanism(AtomicReference<Double> target) {
            trackTarget = position.position(() -> target.get());
        }
    }
}
