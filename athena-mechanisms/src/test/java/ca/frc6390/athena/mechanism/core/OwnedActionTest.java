package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class OwnedActionTest {
    @Test
    void dynamicallyCreatedSoftwareActionUsesItsExplicitMechanismOwner() {
        SoftwareMechanism mechanism = new SoftwareMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create().register(mechanism);

        scheduler.request(mechanism.resetTo(42));
        scheduler.robotPeriodic(0.0, 0.02);

        assertEquals(42, mechanism.value.get());
    }

    @Test
    void ownedActionRunsWhenNestedInASequence() {
        SoftwareMechanism mechanism = new SoftwareMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create().register(mechanism);

        scheduler.request(Actions.sequence()
                .run(mechanism.resetTo(7))
                .run(mechanism.resetTo(11)));
        scheduler.robotPeriodic(0.0, 0.02);
        scheduler.robotPeriodic(0.02, 0.02);

        assertEquals(11, mechanism.value.get());
    }

    @Test
    void externalHookCanLeaseADynamicActionOwnedByAnotherMechanism() {
        boolean[] pressed = {false};
        SoftwareMechanism mechanism = new SoftwareMechanism();
        ExternalControls controls = new ExternalControls(pressed, mechanism);
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .register(controls);

        scheduler.teleopPeriodic(0.0, 0.02);
        pressed[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(23, mechanism.value.get());
    }

    private static final class SoftwareMechanism implements Mechanism {
        private final AtomicInteger value = new AtomicInteger();

        private Action resetTo(int next) {
            return own(Actions.doOnce(() -> value.set(next)));
        }
    }

    private static final class ExternalControls implements Mechanism {
        private final HookBinding reset;

        private ExternalControls(boolean[] pressed, SoftwareMechanism mechanism) {
            reset = Events.when(() -> pressed[0]).active().onStart(mechanism.resetTo(23));
        }
    }
}
