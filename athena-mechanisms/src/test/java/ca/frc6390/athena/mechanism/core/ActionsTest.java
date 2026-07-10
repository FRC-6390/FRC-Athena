package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.List;
import org.junit.jupiter.api.Test;

class ActionsTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

    @Test
    void neutralActionIsShared() {
        assertSame(Actions.neutral(), Actions.neutral());
    }

    @Test
    void builderActionAccessorsReturnStableImmutableViews() {
        Action first = MOTOR.percent(0.2);
        Action second = MOTOR.percent(0.4);
        Actions.Parallel parallel = Actions.parallel(first);
        Actions.Sequence sequence = Actions.sequence().run(first).run(second);
        Actions.Cycle cycle = Actions.cycle().forTime(0.1, first).forTime(0.2, second);

        assertSame(parallel.Actions(), parallel.Actions());
        assertSame(sequence.steps(), sequence.steps());
        assertSame(cycle.steps(), cycle.steps());
        assertThrows(UnsupportedOperationException.class, () -> parallel.Actions().clear());
        assertThrows(UnsupportedOperationException.class, () -> sequence.steps().clear());
        assertThrows(UnsupportedOperationException.class, () -> cycle.steps().clear());
    }

    @Test
    void deadlineActionsAccessorDoesNotCopyEachCall() {
        Action primary = MOTOR.percent(0.2);
        Action other = MOTOR.voltage(3.0);
        Actions.Deadline deadline = Actions.deadline(primary, other);

        List<Action> actions = deadline.Actions();

        assertSame(actions, deadline.Actions());
        assertEquals(List.of(primary, other), actions);
        assertThrows(UnsupportedOperationException.class, () -> actions.clear());
    }

    private record TestMechanism(Action initial) implements Mechanism {
    }
}
