package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class SoftwareActionOwnershipTest {
    @Test
    void declarationFreeActionUsesTheSingleRobotRoot() {
        SoftwareMechanism mechanism = new SoftwareMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create().register(mechanism);

        scheduler.request(mechanism.resetTo(42));
        scheduler.robotPeriodic(0.0, 0.02);

        assertEquals(42, mechanism.value.get());
    }

    @Test
    void externalHookCanLeaseAParameterizedSoftwareActionWithoutOwnershipAnnotations() {
        boolean[] pressed = {false};
        SoftwareMechanism mechanism = new SoftwareMechanism();
        RobotRoot robot = new RobotRoot(mechanism, new ExternalControls(pressed, mechanism));
        MechanismScheduler scheduler = MechanismScheduler.create().register(robot);

        scheduler.teleopPeriodic(0.0, 0.02);
        pressed[0] = true;
        scheduler.teleopPeriodic(0.02, 0.02);

        assertEquals(23, mechanism.value.get());
    }

    @Test
    void ambiguousIndependentRootsStillRejectDeclarationFreeActions() {
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(new SoftwareMechanism())
                .register(new SoftwareMechanism());

        assertThrows(IllegalArgumentException.class,
                () -> scheduler.request(Actions.doOnce(() -> { })));
    }

    private record RobotRoot(SoftwareMechanism mechanism, ExternalControls controls) implements Mechanism {
    }

    private static final class SoftwareMechanism implements Mechanism {
        private final AtomicInteger value = new AtomicInteger();

        private Action resetTo(int next) {
            return Actions.doOnce(() -> value.set(next));
        }
    }

    private static final class ExternalControls implements Mechanism {
        private final HookBinding reset;

        private ExternalControls(boolean[] pressed, SoftwareMechanism mechanism) {
            reset = Events.when(() -> pressed[0]).active().onStart(mechanism.resetTo(23));
        }
    }
}
