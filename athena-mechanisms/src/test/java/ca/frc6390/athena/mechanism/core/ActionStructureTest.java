package ca.frc6390.athena.mechanism.core;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Every composite describes itself and its children once; walkers rely on this structure. */
class ActionStructureTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final ActionCondition NEVER = ctx -> false;

    private record Case(String name, Action action, String label, List<String> roles) {
    }

    private static List<Case> composites() {
        Action leaf = MOTOR.percent(0.2);
        Action other = MOTOR.voltage(Units.Volts.of(3.0));
        ControlBinding<?> position = Controls.position(MOTOR, Units.Rotations);
        return List.of(
                new Case("sequence steps and then",
                        Actions.sequence().run(leaf).run(other).then(leaf),
                        "SEQUENCE", List.of("step", "step", "then")),
                // timeout() wraps what was built so far in a Timeout stage of a new sequence.
                new Case("sequence with timeout",
                        Actions.sequence().run(leaf).timeout(Seconds.of(3.0)),
                        "SEQUENCE", List.of("step")),
                new Case("sequence timeout stage",
                        Actions.sequence().run(leaf).timeout(Seconds.of(3.0)).children().get(0).action(),
                        "TIMEOUT 3.0s", List.of("body")),
                new Case("cycle",
                        Actions.cycle().run(leaf).forTime(Seconds.of(0.1), other),
                        "CYCLE", List.of("step", "step")),
                new Case("parallel",
                        Actions.parallel(leaf, other),
                        "PARALLEL", List.of("branch", "branch")),
                new Case("race",
                        Actions.race(leaf, other),
                        "RACE", List.of("branch", "branch")),
                new Case("deadline",
                        Actions.deadline(leaf, other, leaf),
                        "DEADLINE", List.of("primary", "with", "with")),
                new Case("choice",
                        Actions.when(NEVER).run(leaf).otherwise(other),
                        "CONDITIONAL", List.of("true", "false")),
                new Case("when branch",
                        Actions.when(NEVER).run(leaf),
                        "CONDITIONAL", List.of("true")),
                new Case("timeout",
                        Actions.timeout(leaf, Seconds.of(2.0)),
                        "TIMEOUT 2.0s", List.of("body")),
                new Case("until",
                        Actions.until(NEVER, leaf),
                        "UNTIL", List.of("body")),
                new Case("until then",
                        Actions.until(NEVER, leaf).then(other),
                        "UNTIL", List.of("body", "then")),
                new Case("then",
                        Actions.then(leaf, other),
                        "", List.of("first", "then")),
                new Case("within tolerance",
                        position.position(Units.Rotations.of(1.0)).untilWithin(Units.Rotations.of(0.1)),
                        "WITHIN 0.1", List.of("body")),
                new Case("within tolerance settled and velocity limited",
                        position.position(Units.Rotations.of(1.0)).untilWithin(Units.Rotations.of(0.1)).settled(Seconds.of(0.2)).maxVelocity(Units.RotationsPerSecond.of(3.0)),
                        "WITHIN 0.1 settled 0.2s maxVelocity 3.0", List.of("body")),
                new Case("interruptible",
                        leaf.interruptible(Interrupt.ANYTIME),
                        "INTERRUPT Interrupt.ANYTIME", List.of("body")),
                new Case("effort-limited control",
                        position.position(Units.Rotations.of(1.0)).effort(Units.Amps.of(10)),
                        "POSITION 1.0 effort 10.0A", List.of()),
                new Case("wait",
                        Actions.waitSeconds(Seconds.of(1.5)),
                        "WAIT 1.5s", List.of()),
                new Case("motor leaf",
                        leaf,
                        "MotorPercent", List.of()));
    }

    @Test
    void describesItselfAndItsChildrenInOrder() {
        for (Case c : composites()) {
            assertEquals(c.label(), c.action().describe(), c.name() + " label");
            assertEquals(c.roles(), roles(c.action()), c.name() + " children");
        }
    }

    @Test
    void sequenceReportsEveryStepAndTheContinuation() {
        Action first = MOTOR.percent(0.2);
        Action second = MOTOR.voltage(Units.Volts.of(3.0));
        Action continuation = MOTOR.neutral();
        Actions.Sequence sequence = Actions.sequence().run(first).run(second).then(continuation);

        List<Action> children = sequence.children().stream().map(Action.Child::action).toList();

        assertEquals(List.of(first, second, continuation), children);
    }

    @Test
    void pathMarkersAreChildrenAndTheLabelShowsResetMode() {
        Actions.Timeout marker = (Actions.Timeout) Actions.timeout(MOTOR.percent(0.2), Seconds.of(1.0));
        PathAction path = new PathAction("choreo", "first").marker("collect", marker);

        assertEquals(List.of("marker:collect"), roles(path));
        assertEquals("PATH choreo:first", path.describe());
        assertEquals("PATH choreo:first [reset pose]", path.resetOdometry().describe());
        assertEquals("PATH choreo:first [reset pose only]", path.resetPoseOnly().describe());
    }

    @Test
    void leavesHaveNoChildrenAndNullChildrenAreNeverReported() {
        Action motorPercent = MOTOR.percent(0.2);
        assertTrue(motorPercent.children().isEmpty());
        assertTrue(Actions.neutral().children().isEmpty());

        Action untilOnly = Actions.until(NEVER, MOTOR.percent(0.2));
        assertFalse(untilOnly.children().stream().anyMatch(child -> child.action() == null));
        assertEquals(1, untilOnly.children().size());
        assertEquals(1, Actions.sequence().run(MOTOR.percent(0.2)).children().size());
    }

    @Test
    void childRolesAreNeverNull() {
        List<String> roles = new ArrayList<>();
        roles.addAll(roles(Actions.sequence().run(MOTOR.percent(0.2)).then(MOTOR.neutral())));
        roles.addAll(roles(Actions.deadline(MOTOR.percent(0.2), MOTOR.voltage(Units.Volts.of(1.0)))));
        assertTrue(roles.stream().noneMatch(role -> role == null || role.isBlank()));
    }

    @Test
    void pathIntrospectorFindsPathsThroughEveryCompositeChild() {
        PathAction first = new PathAction("choreo", "first");
        PathAction second = new PathAction("choreo", "second");
        PathAction third = new PathAction("choreo", "third");
        Action root = Actions.sequence()
                .run(Actions.race(first, Actions.until(NEVER, MOTOR.percent(0.2)).then(second)))
                .then(third)
                .interruptible(Interrupt.ANYTIME);

        List<PathAction> found = PathIntrospector.inspect(root);

        assertEquals(List.of(first, second, third), found);
    }

    private static List<String> roles(Action action) {
        return action.children().stream().map(Action.Child::role).toList();
    }
}
