package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

/** Cross-product tests for the scheduler's finite action and group composition contracts. */
class ActionCompositionPermutationTest {
    private static final double PERIOD_SECONDS = 0.02;

    @Test
    void everyRecursiveOperatorAndPositionSignatureCompletesAtTheOracleTick() {
        int depth = 4;
        int[] cases = {0};
        for (TimingPattern timing : TimingPattern.values()) {
            Scenario leaf = Scenario.waiting(timing.ticks(depth));
            enumerateRecursiveSignatures(depth, 0, timing, leaf, scenario -> {
                assertScenarioMatchesOracle(scenario);
                cases[0]++;
            });
        }

        // 4 timing patterns * (4 operators * 2 child positions)^4.
        assertEquals(16_384, cases[0]);
    }

    @Test
    void everyBalancedTwoLevelOperatorTreeAndLeafTimingPermutationMatchesOracle() {
        int cases = 0;
        for (RecursiveOperator root : RecursiveOperator.values()) {
            for (RecursiveOperator leftOperator : RecursiveOperator.values()) {
                for (RecursiveOperator rightOperator : RecursiveOperator.values()) {
                    for (int timingMask = 0; timingMask < 16; timingMask++) {
                        Scenario left = leftOperator.compose(
                                Scenario.waiting(timingMask & 1),
                                Scenario.waiting((timingMask >>> 1) & 1));
                        Scenario right = rightOperator.compose(
                                Scenario.waiting((timingMask >>> 2) & 1),
                                Scenario.waiting((timingMask >>> 3) & 1));

                        assertScenarioMatchesOracle(root.compose(left, right));
                        cases++;
                    }
                }
            }
        }

        // 4 root operators * 4 left operators * 4 right operators * 16 leaf timings.
        assertEquals(1_024, cases);
    }

    @Test
    void everyOrderedGroupAndDurationPermutationHasTheExpectedCompletionTick() {
        for (GroupCase group : GroupCase.values()) {
            for (int firstTicks = 0; firstTicks <= 2; firstTicks++) {
                for (int secondTicks = 0; secondTicks <= 2; secondTicks++) {
                    RecordingPath first = new RecordingPath("first", firstTicks * PERIOD_SECONDS);
                    RecordingPath second = new RecordingPath("second", secondTicks * PERIOD_SECONDS);
                    Action composition = group.compose(first.action, second.action);
                    MechanismRuntime runtime = runtime(composition, first, second);
                    int expectedTick = group.completionTick(firstTicks, secondTicks);

                    for (int tick = 0; tick <= 3; tick++) {
                        run(runtime, tick);
                        assertEquals(tick >= expectedTick, runtime.traceSnapshot().schedulerComplete(),
                                group + " completion mismatch for [" + firstTicks + ", " + secondTicks
                                        + "] at tick " + tick);
                    }

                    assertEquals(1, first.normalEnds + first.interruptions,
                            group + " did not terminate first child exactly once");
                    assertEquals(1, second.normalEnds + second.interruptions,
                            group + " did not terminate second child exactly once");
                    assertEquals(group.firstInterrupted(firstTicks, secondTicks), first.interruptions == 1);
                    assertEquals(group.secondInterrupted(firstTicks, secondTicks), second.interruptions == 1);
                }
            }
        }
    }

    @Test
    void everyFiniteWrapperWorksInEveryGroupPosition() {
        List<FiniteCase> finiteCases = finiteCases();
        for (FiniteCase finite : finiteCases) {
            for (GroupCase group : GroupCase.values()) {
                for (int position = 0; position < 2; position++) {
                    Action wrapped = finite.factory.get();
                    Action companion = Actions.waitSeconds(PERIOD_SECONDS);
                    Action composition = position == 0
                            ? group.compose(wrapped, companion)
                            : group.compose(companion, wrapped);
                    MechanismRuntime runtime = runtime(composition);

                    run(runtime, 0);
                    run(runtime, 1);

                    assertTrue(runtime.traceSnapshot().schedulerComplete(),
                            finite.name + " failed in " + group + " position " + position);
                }
            }
        }
    }

    @Test
    void everyNestedGroupPermutationCompletesWithoutLeakingChildState() {
        for (GroupCase outer : GroupCase.values()) {
            for (GroupCase inner : GroupCase.values()) {
                for (int innerPosition = 0; innerPosition < 2; innerPosition++) {
                    Action innerGroup = inner.compose(
                            Actions.waitSeconds(0.0),
                            Actions.waitSeconds(PERIOD_SECONDS));
                    Action outerCompanion = Actions.waitSeconds(2.0 * PERIOD_SECONDS);
                    Action composition = innerPosition == 0
                            ? outer.compose(innerGroup, outerCompanion)
                            : outer.compose(outerCompanion, innerGroup);
                    MechanismRuntime runtime = runtime(composition);

                    for (int tick = 0; tick <= 3; tick++) run(runtime, tick);

                    assertTrue(runtime.traceSnapshot().schedulerComplete(),
                            outer + " containing " + inner + " in position " + innerPosition
                                    + " failed to complete");
                }
            }
        }
    }

    @Test
    void repeatedActionInstanceHasIndependentOccurrenceState() {
        int[] calls = {0};
        Action shared = Actions.doOnce(() -> calls[0]++);
        MechanismRuntime parallel = runtime(Actions.parallel(shared, shared));

        run(parallel, 0);
        run(parallel, 1);

        assertTrue(parallel.traceSnapshot().schedulerComplete());
        assertEquals(2, calls[0], "each structural occurrence must initialize once");
    }

    @Test
    void cancellationOfEveryGroupInterruptsEachRunningChildOnce() {
        for (GroupCase group : GroupCase.values()) {
            RecordingPath first = new RecordingPath("first", 10.0);
            RecordingPath second = new RecordingPath("second", 10.0);
            Action composition = group.compose(first.action, second.action);
            MechanismRuntime runtime = runtime(composition, first, second);

            runtime.periodic(contextAt(0), EventContext.empty());
            runtime.set(Actions.neutral());
            runtime.periodic(contextAt(1), EventContext.empty());

            assertEquals(1, first.interruptions, group + " did not interrupt first child once");
            assertEquals(1, second.interruptions, group + " did not interrupt second child once");
            assertEquals(0, first.normalEnds + second.normalEnds);
        }
    }

    private static List<FiniteCase> finiteCases() {
        Action stableWait = Actions.waitSeconds(PERIOD_SECONDS);
        return List.of(
                new FiniteCase("doOnce", () -> Actions.doOnce(() -> { })),
                new FiniteCase("waitSeconds", () -> Actions.waitSeconds(PERIOD_SECONDS)),
                new FiniteCase("waitUntil", () -> Actions.waitUntil(ctx -> ctx.timeInStateSeconds() >= PERIOD_SECONDS)),
                new FiniteCase("timeout", () -> Actions.timeout(Actions.neutral(), PERIOD_SECONDS)),
                new FiniteCase("until", () -> Actions.until(
                        ctx -> ctx.timeInStateSeconds() >= PERIOD_SECONDS, Actions.neutral())),
                new FiniteCase("then", () -> Actions.then(Actions.doOnce(() -> { }),
                        Actions.waitSeconds(PERIOD_SECONDS))),
                new FiniteCase("sequence", () -> Actions.sequence()
                        .doOnce(() -> { })
                        .waitSeconds(PERIOD_SECONDS)),
                new FiniteCase("choice", () -> Actions.when(() -> true)
                        .run(Actions.waitSeconds(PERIOD_SECONDS))
                        .otherwise(Actions.neutral().timeout(PERIOD_SECONDS))),
                new FiniteCase("computed", () -> Actions.compute(() -> stableWait, new Object())),
                new FiniteCase("hardwareComputed", () -> Actions.computeHardware(List.of(), ignored -> stableWait)));
    }

    private static void enumerateRecursiveSignatures(
            int remainingDepth,
            int level,
            TimingPattern timing,
            Scenario nested,
            Consumer<Scenario> consumer) {
        if (remainingDepth == 0) {
            consumer.accept(nested);
            return;
        }
        Scenario companion = Scenario.waiting(timing.ticks(level));
        for (RecursiveOperator operator : RecursiveOperator.values()) {
            enumerateRecursiveSignatures(
                    remainingDepth - 1,
                    level + 1,
                    timing,
                    operator.compose(nested, companion),
                    consumer);
            enumerateRecursiveSignatures(
                    remainingDepth - 1,
                    level + 1,
                    timing,
                    operator.compose(companion, nested),
                    consumer);
        }
    }

    private static void assertScenarioMatchesOracle(Scenario scenario) {
        MechanismRuntime runtime = runtime(scenario.action);
        for (int tick = 0; tick <= scenario.completionTick + 1; tick++) {
            run(runtime, tick);
            assertEquals(
                    tick >= scenario.completionTick,
                    runtime.traceSnapshot().schedulerComplete(),
                    scenario.description + " completion mismatch at tick " + tick
                            + "; expected " + scenario.completionTick);
        }
    }

    private static MechanismRuntime runtime(Action action, RecordingPath... paths) {
        MechanismRuntime runtime = MechanismRuntime.of(new TestMechanism(action), ActionContext.empty());
        for (RecordingPath path : paths) runtime.path(path.action, path);
        runtime.traceLevel(MechanismTraceLevel.SUMMARY);
        runtime.tracePeriodSeconds(0.0);
        runtime.set(action);
        return runtime;
    }

    private static void run(MechanismRuntime runtime, int tick) {
        runtime.periodic(contextAt(tick), EventContext.empty());
    }

    private static MechanismContext contextAt(int tick) {
        return new MechanismContext(tick * PERIOD_SECONDS, 0.0, PERIOD_SECONDS, true, false, false);
    }

    private record TestMechanism(Action action) implements Mechanism {
    }

    private record FiniteCase(String name, Supplier<Action> factory) {
    }

    private record Scenario(Action action, int completionTick, String description) {
        private static Scenario waiting(int ticks) {
            int safeTicks = Math.max(0, ticks);
            return new Scenario(
                    Actions.waitSeconds(safeTicks * PERIOD_SECONDS),
                    safeTicks,
                    "wait(" + safeTicks + ")");
        }
    }

    private enum TimingPattern {
        ALL_IMMEDIATE {
            @Override int ticks(int level) { return 0; }
        },
        ALL_DELAYED {
            @Override int ticks(int level) { return 1; }
        },
        EVEN_DELAYED {
            @Override int ticks(int level) { return level % 2 == 0 ? 1 : 0; }
        },
        ODD_DELAYED {
            @Override int ticks(int level) { return level % 2 == 0 ? 0 : 1; }
        };

        abstract int ticks(int level);
    }

    private enum RecursiveOperator {
        SEQUENCE {
            @Override Scenario compose(Scenario first, Scenario second) {
                return scenario(
                        Actions.sequence().run(first.action).then(second.action),
                        first.completionTick + second.completionTick,
                        first,
                        second);
            }
        },
        PARALLEL {
            @Override Scenario compose(Scenario first, Scenario second) {
                return scenario(
                        Actions.parallel(first.action, second.action),
                        Math.max(first.completionTick, second.completionTick),
                        first,
                        second);
            }
        },
        RACE {
            @Override Scenario compose(Scenario first, Scenario second) {
                return scenario(
                        Actions.race(first.action, second.action),
                        Math.min(first.completionTick, second.completionTick),
                        first,
                        second);
            }
        },
        DEADLINE {
            @Override Scenario compose(Scenario first, Scenario second) {
                return scenario(
                        Actions.deadline(first.action, second.action),
                        first.completionTick,
                        first,
                        second);
            }
        };

        abstract Scenario compose(Scenario first, Scenario second);

        Scenario scenario(Action action, int tick, Scenario first, Scenario second) {
            return new Scenario(
                    action,
                    tick,
                    name().toLowerCase() + "(" + first.description + "," + second.description + ")");
        }
    }

    private enum GroupCase {
        PARALLEL(Actions::parallel) {
            @Override int completionTick(int first, int second) { return Math.max(first, second); }
            @Override boolean firstInterrupted(int first, int second) { return false; }
            @Override boolean secondInterrupted(int first, int second) { return false; }
        },
        RACE(Actions::race) {
            @Override int completionTick(int first, int second) { return Math.min(first, second); }
            @Override boolean firstInterrupted(int first, int second) { return first > second; }
            @Override boolean secondInterrupted(int first, int second) { return second > first; }
        },
        DEADLINE(Actions::deadline) {
            @Override int completionTick(int first, int second) { return first; }
            @Override boolean firstInterrupted(int first, int second) { return false; }
            @Override boolean secondInterrupted(int first, int second) { return second > first; }
        };

        private final BiFunction<Action, Action, Action> composition;

        GroupCase(BiFunction<Action, Action, Action> composition) {
            this.composition = composition;
        }

        Action compose(Action first, Action second) {
            return composition.apply(first, second);
        }

        abstract int completionTick(int first, int second);
        abstract boolean firstInterrupted(int first, int second);
        abstract boolean secondInterrupted(int first, int second);
    }

    private static final class RecordingPath implements PathRuntime {
        private final PathAction action;
        private final double durationSeconds;
        private int normalEnds;
        private int interruptions;

        private RecordingPath(String name, double durationSeconds) {
            this.action = Paths.of("permutation", name + "-" + durationSeconds + "-" + System.identityHashCode(this));
            this.durationSeconds = durationSeconds;
        }

        @Override
        public boolean isFinished(PathAction path, MechanismContext context) {
            return context.timeInStateSeconds() >= durationSeconds;
        }

        @Override
        public void end(PathAction path, MechanismContext context, boolean interrupted) {
            if (interrupted) interruptions++;
            else normalEnds++;
        }
    }
}
