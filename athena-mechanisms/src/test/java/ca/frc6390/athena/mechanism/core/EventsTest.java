package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class EventsTest {
    @Test
    void robotPeriodicIsAnUmbrellaForEveryModePeriodicPass() {
        EventBinding robotPeriodic = Events.robotPeriodic();

        assertTrue(robotPeriodic.sourceActive(context(LifecycleMode.DISABLED, LifecyclePhase.PERIODIC)));
        assertTrue(robotPeriodic.sourceActive(context(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC)));
        assertTrue(robotPeriodic.sourceActive(context(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC)));
        assertTrue(robotPeriodic.sourceActive(context(LifecycleMode.TEST, LifecyclePhase.PERIODIC)));
        assertFalse(robotPeriodic.sourceActive(context(LifecycleMode.TELEOP, LifecyclePhase.INIT)));
    }

    @Test
    void modePeriodicEventsRemainModeSpecific() {
        EventBinding teleopPeriodic = Events.teleopPeriodic();

        assertTrue(teleopPeriodic.sourceActive(context(LifecycleMode.TELEOP, LifecyclePhase.PERIODIC)));
        assertFalse(teleopPeriodic.sourceActive(context(LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC)));
    }

    private static EventContext context(LifecycleMode mode, LifecyclePhase phase) {
        boolean enabled = mode != LifecycleMode.DISABLED;
        return new EventContext(1.0, 0.02, mode, phase, enabled, false);
    }
}
