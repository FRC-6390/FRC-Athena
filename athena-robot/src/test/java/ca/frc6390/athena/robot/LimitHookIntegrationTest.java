package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.sim.hardware.SimMotorHandle.CommandKind;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

class LimitHookIntegrationTest {
    @Test
    void heldInvertedHomeSwitchZerosEveryCycleAndReportsLifecycleOnce() {
        Rig rig = new Rig();
        rig.rawHome(false);
        rig.motorState(4.5);

        rig.runtime.disabledPeriodic(0.0, 0.02);
        assertEquals(4.5, rig.motorPosition(), 1.0e-9);
        assertEquals(0, rig.arm.homeStarts);
        assertEquals(0, rig.arm.homeEnds);

        rig.rawHome(true);
        rig.runtime.disabledPeriodic(0.02, 0.02);
        assertEquals(0.0, rig.motorPosition(), 1.0e-9);
        assertEquals(1, rig.arm.homeStarts);

        rig.motorState(2.0);
        rig.runtime.disabledPeriodic(0.04, 0.02);
        assertEquals(0.0, rig.motorPosition(), 1.0e-9);
        assertEquals(1, rig.arm.homeStarts);

        rig.rawHome(false);
        rig.motorState(3.0);
        rig.runtime.disabledPeriodic(0.06, 0.02);
        assertEquals(3.0, rig.motorPosition(), 1.0e-9);
        assertEquals(1, rig.arm.homeEnds);
    }

    @Test
    void homeZeroingRunsBeforeAnActivePositionControlReadsFeedback() {
        Rig rig = new Rig();
        rig.rawHome(false);
        rig.motorState(4.5);
        rig.runtime.request(rig.arm.holdHome);
        rig.runtime.teleopPeriodic(0.0, 0.02);

        rig.rawHome(true);
        rig.runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(0.0, rig.motorPosition(), 1.0e-9);
        assertEquals(CommandKind.VOLTAGE, rig.motor().commandKind());
        assertEquals(0.0, rig.motor().commandValue(), 1.0e-9);
    }

    @Test
    void homingActionNeutralizesAsSoonAsTheHomeSwitchBecomesActive() {
        Rig rig = new Rig();
        rig.rawHome(false);
        rig.runtime.request(rig.arm.homeAction);
        rig.runtime.teleopPeriodic(0.0, 0.02);
        assertEquals(CommandKind.PERCENT, rig.motor().commandKind());
        assertEquals(-0.4, rig.motor().commandValue(), 1.0e-9);

        rig.rawHome(true);
        rig.runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(CommandKind.NEUTRAL, rig.motor().commandKind());
        assertEquals(0.0, rig.motor().commandValue(), 1.0e-9);
    }

    @Test
    void homeZeroingResetsDerivativeStateBeforeControlResumes() {
        Rig rig = new Rig();
        rig.motorState(rig.arm.encoder.rotationsFromPosition(0.2));
        rig.runtime.request(rig.arm.derivativeTarget);
        rig.runtime.teleopPeriodic(0.0, 0.02);
        assertEquals(0.0, rig.motor().commandValue(), 1.0e-9);

        rig.rawHome(true);
        rig.runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(0.0, rig.motorPosition(), 1.0e-9);
        assertEquals(0.0, rig.motor().commandValue(), 1.0e-9);
    }

    @Test
    void disableResetsDerivativeStateBeforeControlResumes() {
        Rig rig = new Rig();
        rig.motorState(rig.arm.encoder.rotationsFromPosition(0.2));
        rig.runtime.request(rig.arm.derivativeTarget);
        rig.runtime.teleopPeriodic(0.0, 0.02);
        rig.runtime.disabledPeriodic(0.02, 0.02);

        rig.motorState(0.0);
        rig.runtime.teleopPeriodic(0.04, 0.02);

        assertEquals(CommandKind.VOLTAGE, rig.motor().commandKind());
        assertEquals(0.0, rig.motor().commandValue(), 1.0e-9);
    }

    @Test
    void rangeConstraintBlocksMotionPastLowerLimitButAllowsRecovery() {
        Rig rig = new Rig();
        rig.motorState(0.0);
        rig.runtime.request(rig.arm.manualIn);
        rig.runtime.teleopPeriodic(0.0, 0.02);
        assertEquals(CommandKind.NEUTRAL, rig.motor().commandKind());

        rig.runtime.request(rig.arm.manualOut);
        rig.runtime.teleopPeriodic(0.02, 0.02);
        assertEquals(CommandKind.PERCENT, rig.motor().commandKind());
        assertEquals(0.2, rig.motor().commandValue(), 1.0e-9);
    }

    @Test
    void rangeConstraintBlocksMotionPastUpperLimitButAllowsRecovery() {
        Rig rig = new Rig();
        rig.motorState(rig.arm.encoder.rotationsFromPosition(0.35));
        assertEquals(0.35, rig.encoderPosition(), 1.0e-9);
        rig.runtime.request(rig.arm.manualOut);
        rig.runtime.teleopPeriodic(0.0, 0.02);
        assertEquals(CommandKind.NEUTRAL, rig.motor().commandKind());

        rig.runtime.request(rig.arm.manualIn);
        rig.runtime.teleopPeriodic(0.02, 0.02);
        assertEquals(CommandKind.PERCENT, rig.motor().commandKind());
        assertEquals(-0.2, rig.motor().commandValue(), 1.0e-9);
    }

    @Test
    void rangeConstraintClampsPositionGoalBeforePidCalculatesVoltage() {
        Rig rig = new Rig();
        rig.motorState(rig.arm.encoder.rotationsFromPosition(0.1));
        assertEquals(0.1, rig.encoderPosition(), 1.0e-9);
        rig.runtime.request(rig.arm.outsideRange);

        rig.runtime.teleopPeriodic(0.0, 0.02);

        assertEquals(CommandKind.VOLTAGE, rig.motor().commandKind());
        assertEquals(0.125, rig.motor().commandValue(), 1.0e-9);
    }

    private static final class Rig {
        private final SimulationSession simulation = SimulationSession.create();
        private final HomeLimitedArm arm = new HomeLimitedArm();
        private final RobotRuntime runtime = RobotRuntime.simulated(simulation).register(arm);

        private Rig() {
            rawHome(false);
        }

        private void rawHome(boolean active) {
            simulation.digitalInput(arm.home).raw(!active);
        }

        private void motorState(double positionRotations) {
            motor().state(positionRotations, 0.0);
        }

        private double motorPosition() {
            return motor().integratedPositionRotations();
        }

        private double encoderPosition() {
            return arm.encoder.positionFromRotations(
                    runtime.hardwareGraph().encoder(arm.encoder).positionRotations());
        }

        private ca.frc6390.athena.sim.hardware.SimMotorHandle motor() {
            return simulation.motor(arm.motor);
        }
    }

    private static final class HomeLimitedArm implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 58);
        private final EncoderDevice encoder = motor.encoder().gearRatio(1.0 / 9.0);
        private final DigitalInputDevice home = DigitalInputDevice.rio(8).inverted();
        private final ControlBinding position = Controls.position(motor)
                .feedback(encoder)
                .pid(0.5, 0.0, 0.0)
                .constraint(Constraints.range(Range.of(0.0, 0.35)));
        private final Action holdHome = position.position(0.0);
        private final Action homeAction = Actions.sequence()
                .until(home::active, motor.percent(-0.4));
        private final Action manualIn = position.percent(-0.2);
        private final Action manualOut = position.percent(0.2);
        private final Action outsideRange = position.position(1.0);
        private final ControlBinding derivativePosition = Controls.position(motor)
                .feedback(encoder)
                .pid(0.0, 0.0, 1.0)
                .constraint(Constraints.range(Range.of(0.0, 0.35)));
        private final Action derivativeTarget = derivativePosition.position(0.2);
        private int homeStarts;
        private int homeEnds;
        private final HookBinding zeroAtHome = Events.when(home).active()
                .onStart(() -> homeStarts++)
                .whileActive(encoder.setPosition(0.0))
                .onEnd(() -> homeEnds++);
    }
}
