package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

class MotorStallIntegrationTest {
    @Test
    void currentThresholdTracksTheConfiguredMotorLimit() {
        MotorDevice fortyAmpMotor = MotorDevice.of(MotorKinds.KRAKEN_X44, 70).statorCurrentLimit(40);
        MotorDevice sixtyAmpMotor = MotorDevice.of(MotorKinds.KRAKEN_X44, 70).statorCurrentLimit(60);

        assertEquals(36.0, fortyAmpMotor.stall().currentThresholdAmps(), 1.0e-9);
        assertEquals(54.0, sixtyAmpMotor.stall().currentThresholdAmps(), 1.0e-9);
        assertEquals(0, fortyAmpMotor.currentLimits().supplyAmps());
        assertEquals(60, sixtyAmpMotor.currentLimits().statorAmps());
    }

    @Test
    void lockedIntakeReversesThenResumesItsHeldAction() {
        SimulationSession simulation = SimulationSession.create();
        JamClearingIntake intake = new JamClearingIntake();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(intake);

        runtime.teleopPeriodic(0.00, 0.02);
        assertEquals(0.8, simulation.motor(intake.roller).commandValue(), 1.0e-9);

        simulation.motor(intake.roller).locked(true);
        runtime.teleopPeriodic(0.02, 0.02);
        runtime.teleopPeriodic(0.04, 0.02);
        runtime.teleopPeriodic(0.06, 0.02);
        runtime.teleopPeriodic(0.08, 0.02);
        assertEquals(-0.6, simulation.motor(intake.roller).commandValue(), 1.0e-9);

        simulation.motor(intake.roller).locked(false);
        runtime.teleopPeriodic(0.18, 0.10);
        runtime.teleopPeriodic(0.24, 0.06);
        runtime.teleopPeriodic(0.26, 0.02);
        runtime.teleopPeriodic(0.30, 0.04);
        runtime.teleopPeriodic(0.32, 0.02);
        assertEquals(0.8, simulation.motor(intake.roller).commandValue(), 1.0e-9);
    }

    private static final class JamClearingIntake implements Mechanism {
        private final MotorDevice roller = MotorDevice.of(MotorKinds.KRAKEN_X44, 71)
                .supplyCurrentLimit(35)
                .statorCurrentLimit(50);
        private final Action intake = roller.percent(0.8);
        private final Action clearJam = Actions.sequence()
                .forTime(0.12, roller.percent(-0.6))
                .forTime(0.04, roller.neutral());
        private final HookBinding run = Events.teleopPeriodic().whileActive(intake);
        private final HookBinding recover = Events.when(roller.stall().forSeconds(0.06))
                .rising()
                .onStart(clearJam);
    }
}
