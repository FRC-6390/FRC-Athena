package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class SequenceReuseTest {
    @Test
    void appendingReturnsIndependentSequences() {
        Actions.Sequence base = Actions.sequence().doOnce(() -> { });
        Action leftEnd = Actions.doOnce(() -> { });
        Action rightEnd = Actions.doOnce(() -> { });

        Actions.Sequence left = base.then(leftEnd);
        Actions.Sequence right = base.then(rightEnd);

        assertNotSame(base, left);
        assertNotSame(base, right);
        assertNotSame(left, right);
        assertNull(base.next());
        assertSame(leftEnd, left.next());
        assertSame(rightEnd, right.next());
        assertEquals(base.steps(), left.steps());
    }

    @Test
    void appendingTheSameSequenceTwiceDoesNotCreateASelfReference() {
        Actions.Sequence repeat = Actions.sequence().waitSeconds(0.02);
        Action collect = Actions.doOnce(() -> { });

        assertDoesNotThrow(() -> Actions.sequence().then(repeat).then(repeat).then(collect));

        assertNull(repeat.next());
    }

    @Test
    void appendedCopyPreservesTimeoutAndIndependentlyCarriesStartedActions() {
        Action background = Actions.waitSeconds(1.0);
        Action firstEnd = Actions.doOnce(() -> { });
        Action secondEnd = Actions.doOnce(() -> { });
        Actions.Sequence base = Actions.sequence().start(background).waitSeconds(0.1).timeout(0.5);

        Actions.Sequence first = base.then(firstEnd);
        first.stop(background);
        Actions.Sequence second = base.then(secondEnd);

        assertEquals(0.5, first.timeoutSeconds());
        assertEquals(base.steps(), first.steps());
        Actions.Deadline firstFollowup = (Actions.Deadline) first.next();
        Actions.Deadline secondFollowup = (Actions.Deadline) second.next();
        assertSame(firstEnd, firstFollowup.primary());
        assertSame(secondEnd, secondFollowup.primary());
        assertEquals(List.of(background), firstFollowup.others());
        assertEquals(List.of(background), secondFollowup.others());
        assertNull(base.next());
    }

    @Test
    void repeatedSequenceExecutesBothOccurrencesAndTheFinalFollowupInOrder() {
        List<String> events = new ArrayList<>();
        Actions.Sequence repeat = Actions.sequence()
                .doOnce(() -> events.add("repeat"))
                .waitSeconds(0.02);
        Action action = Actions.sequence()
                .doOnce(() -> events.add("before"))
                .then(repeat)
                .then(repeat)
                .then(Actions.doOnce(() -> events.add("after")));
        MechanismRuntime runtime = MechanismRuntime.of(new TestMechanism(action), ActionContext.empty());
        runtime.set(action);

        for (int index = 0; index < 10; index++) {
            runtime.periodic(new MechanismContext(index * 0.02, 0.0, 0.02, true, false, false),
                    EventContext.empty());
        }

        assertEquals(List.of("before", "repeat", "repeat", "after"), events);
        assertNull(repeat.next());
    }

    private record TestMechanism(Action initial) implements Mechanism {
    }
}
