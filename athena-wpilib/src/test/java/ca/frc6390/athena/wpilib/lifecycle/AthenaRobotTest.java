package ca.frc6390.athena.wpilib.lifecycle;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.ActionRequests;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

class AthenaRobotTest {
    @Test
    void mapsWpilibLifecycleCallbacksToAthenaContexts() {
        ManualClock clock = new ManualClock(10.0);
        List<Call> calls = new ArrayList<>();
        AthenaRobotLifecycle lifecycle = lifecycle(clock, calls);

        clock.set(10.02);
        lifecycle.robotInit();
        clock.set(10.05);
        lifecycle.disabledPeriodic();
        clock.set(10.07);
        lifecycle.autonomousPeriodic();
        clock.set(10.10);
        lifecycle.simulationPeriodic();

        assertEquals(3, calls.size());
        assertCall(calls.get(0), LifecycleMode.ROBOT, LifecyclePhase.INIT, true, false, false, 0.02);
        assertCall(calls.get(1), LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false, false, 0.03);
        assertCall(calls.get(2), LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, true, false, 0.02);
    }

    @Test
    void clampsNegativeAndNonFiniteDtToZero() {
        ManualClock clock = new ManualClock(5.0);
        List<Call> calls = new ArrayList<>();
        AthenaRobotLifecycle lifecycle = lifecycle(clock, calls);

        clock.set(4.0);
        lifecycle.teleopPeriodic();
        clock.set(Double.NaN);
        lifecycle.testPeriodic();

        assertEquals(0.0, calls.get(0).mechanismContext.dtSeconds());
        assertEquals(0.0, calls.get(1).mechanismContext.dtSeconds());
        assertEquals(0.0, calls.get(1).eventContext.dtSeconds());
    }

    @Test
    void createsSimulationRuntimeOnlyWhenWpilibReportsSimulation() {
        assertNotNull(AthenaRobot.createRuntime(true).simulationSession());
        assertNull(AthenaRobot.createRuntime(false).simulationSession());
    }

    @Test
    void closingRuntimeClearsGlobalActionRequests() {
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create());
        ActionRequests.bind(runtime::request);
        try {
            AthenaRobot.closeRuntime(runtime);

            assertThrows(IllegalStateException.class, () -> ActionRequests.request(Actions.neutral()));
            assertThrows(IllegalStateException.class, () -> runtime.request(Actions.neutral()));
        } finally {
            ActionRequests.clear();
            runtime.close();
        }
    }

    private static AthenaRobotLifecycle lifecycle(ManualClock clock, List<Call> calls) {
        return new AthenaRobotLifecycle(clock, (mechanismContext, eventContext) ->
                calls.add(new Call(mechanismContext, eventContext)));
    }

    private static void assertCall(
            Call call,
            LifecycleMode mode,
            LifecyclePhase phase,
            boolean enabled,
            boolean autonomous,
            boolean simulation,
            double dtSeconds) {
        assertEquals(mode, call.eventContext.mode());
        assertEquals(phase, call.eventContext.phase());
        assertEquals(enabled, call.eventContext.enabled());
        assertEquals(simulation, call.eventContext.simulation());
        assertEquals(dtSeconds, call.eventContext.dtSeconds(), 1.0e-9);
        assertEquals(dtSeconds, call.mechanismContext.dtSeconds(), 1.0e-9);
        assertEquals(enabled, call.mechanismContext.enabled());
        assertEquals(autonomous, call.mechanismContext.autonomous());
        assertEquals(simulation, call.mechanismContext.simulation());
        if (enabled) {
            assertTrue(call.mechanismContext.enabled());
        } else {
            assertFalse(call.mechanismContext.enabled());
        }
    }

    private record Call(MechanismContext mechanismContext, EventContext eventContext) {
    }

    private static final class ManualClock implements DoubleSupplier {
        private double timestampSeconds;

        private ManualClock(double timestampSeconds) {
            this.timestampSeconds = timestampSeconds;
        }

        @Override
        public double getAsDouble() {
            return timestampSeconds;
        }

        private void set(double timestampSeconds) {
            this.timestampSeconds = timestampSeconds;
        }
    }

}
