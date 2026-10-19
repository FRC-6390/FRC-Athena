package ca.frc6390.athena.mechanism.core;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * Differential fuzzing of action composition and lease arbitration. Random finite action trees are
 * run through the real scheduler and compared tick by tick against an independent timing model of
 * the documented semantics: then/sequence run stages back to back, parallel ends with its slowest
 * child, race with its fastest, deadline with its first, timeout/forTime/until cut a child off, and
 * either/onlyIf pick a branch once at start.
 */
class ActionFuzzTest {
    private static final double DT = 0.02;
    private static final MotorDevice[] MOTORS = {
        MotorDevice.of(MotorKinds.KRAKEN_X60, 51),
        MotorDevice.of(MotorKinds.KRAKEN_X60, 52),
        MotorDevice.of(MotorKinds.KRAKEN_X44, 53),
        MotorDevice.of(MotorKinds.KRAKEN_X44, 54),
    };
    private static final int CASES = 1_500;

    @Test
    void singleLeaseMatchesTheTimingModel() {
        Random random = new Random(6390);
        for (int index = 0; index < CASES; index++) {
            Node tree = generate(random, 4, allMotors());
            Rig rig = new Rig();
            Action action = tree.build(rig);
            rig.scheduler.request(action);
            Expectation expected = new Expectation();
            tree.expect(0, tree.duration(), expected);
            int horizon = tree.duration() + 3;
            for (int tick = 0; tick < horizon; tick++) {
                rig.tick(tick);
                expected.assertTick(rig, tick, tree, null);
            }
            expected.assertDoOnces(tree);
            assertEquals(true, rig.scheduler.isComplete(action), "did not report completion: " + tree);
        }
    }

    @Test
    void restartingAnActionReplaysItFromTheStart() {
        Random random = new Random(254);
        for (int index = 0; index < CASES / 3; index++) {
            Node tree = generate(random, 3, allMotors());
            Rig rig = new Rig();
            Action action = tree.build(rig);
            int restartTick = 1 + random.nextInt(Math.max(1, tree.duration() + 1));
            Expectation expected = new Expectation();
            tree.expect(restartTick, restartTick + tree.duration(), expected);
            rig.scheduler.request(action);
            for (int tick = 0; tick < restartTick + tree.duration() + 3; tick++) {
                if (tick == restartTick) {
                    rig.counters.clear();
                    rig.scheduler.request(action);
                }
                rig.tick(tick);
                if (tick >= restartTick) expected.assertTick(rig, tick, tree, "after restart");
            }
            expected.assertDoOnces(tree);
        }
    }

    @Test
    void disabledTimeDoesNotAdvanceActionTimers() {
        Random random = new Random(1114);
        for (int index = 0; index < CASES / 3; index++) {
            Node tree = generate(random, 3, allMotors());
            Rig rig = new Rig();
            rig.scheduler.request(tree.build(rig));
            int pauseAt = random.nextInt(Math.max(1, tree.duration()));
            int pauseTicks = 1 + random.nextInt(40);
            Expectation expected = new Expectation();
            tree.expect(0, tree.duration(), expected);
            int enabledTick = 0;
            int paused = 0;
            for (int clock = 0; enabledTick < tree.duration() + 3; clock++) {
                if (enabledTick == pauseAt && paused < pauseTicks) {
                    paused++;
                    rig.scheduler.disabledPeriodic(clock * DT, DT);
                    continue;
                }
                rig.tick(clock);
                expected.assertTick(rig, enabledTick, tree, "after a " + pauseTicks + "-tick disable");
                enabledTick++;
            }
        }
    }

    @Test
    void independentLeasesNeverDisturbEachOther() {
        Random random = new Random(42);
        for (int index = 0; index < CASES / 2; index++) {
            List<MotorDevice> left = List.of(MOTORS[0], MOTORS[1]);
            List<MotorDevice> right = List.of(MOTORS[2], MOTORS[3]);
            Node first = generate(random, 3, left);
            Node second = generate(random, 3, right);
            Rig rig = new Rig();
            int secondStart = random.nextInt(4);
            Expectation expected = new Expectation();
            first.expect(0, first.duration(), expected);
            second.expect(secondStart, secondStart + second.duration(), expected);
            Action firstAction = first.build(rig);
            Action secondAction = second.build(rig);
            int horizon = Math.max(first.duration(), secondStart + second.duration()) + 3;
            for (int tick = 0; tick < horizon; tick++) {
                if (tick == 0) rig.scheduler.request(firstAction);
                if (tick == secondStart) rig.scheduler.request(secondAction);
                rig.tick(tick);
                expected.assertTick(rig, tick, first, "alongside " + second);
            }
        }
    }

    @Test
    void newestLeaseWinsSharedMotorsAndOlderLeaseResumes() {
        Random random = new Random(7);
        for (int index = 0; index < CASES / 2; index++) {
            Node older = generate(random, 3, allMotors());
            Node newer = generate(random, 3, List.of(MOTORS[0], MOTORS[1]));
            Rig rig = new Rig();
            int newerStart = random.nextInt(Math.max(1, older.duration()));
            Expectation olderExpected = new Expectation();
            older.expect(0, older.duration(), olderExpected);
            Expectation newerExpected = new Expectation();
            newer.expect(newerStart, newerStart + newer.duration(), newerExpected);
            Action olderAction = older.build(rig);
            Action newerAction = newer.build(rig);
            int newerEnd = newerStart + newer.duration();
            int horizon = Math.max(older.duration(), newerEnd) + 3;
            for (int tick = 0; tick < horizon; tick++) {
                if (tick == 0) rig.scheduler.request(olderAction);
                if (tick == newerStart) rig.scheduler.request(newerAction);
                rig.tick(tick);
                for (MotorDevice motor : MOTORS) {
                    Double newerValue = newerExpected.value(tick, motor);
                    boolean newerOwnsMotor = motor == MOTORS[0] || motor == MOTORS[1];
                    Double want;
                    if (newerValue != null) {
                        want = newerValue;
                    } else if (!newerOwnsMotor || tick < newerStart || tick >= newerEnd) {
                        want = olderExpected.value(tick, motor);
                    } else {
                        continue; // the newer lease may reserve the motor between its own outputs
                    }
                    double actual = rig.percent(motor);
                    if (Math.abs(actual - (want == null ? 0.0 : want)) > 1e-9) {
                        fail("tick " + tick + " motor " + motor.id() + " expected " + want + " got " + actual
                                + "\n older=" + older + "\n newer@" + newerStart + "=" + newer);
                    }
                }
            }
        }
    }

    @Test
    void cancellingALeaseReleasesItsMotorsAndChildren() {
        Random random = new Random(99);
        for (int index = 0; index < CASES / 3; index++) {
            Node tree = generate(random, 3, allMotors());
            Rig rig = new Rig();
            Action action = tree.build(rig);
            rig.scheduler.request(action);
            int cancelTick = random.nextInt(Math.max(1, tree.duration()));
            for (int tick = 0; tick < tree.duration() + 3; tick++) {
                if (tick == cancelTick) rig.scheduler.cancel(action);
                rig.tick(tick);
                if (tick >= cancelTick) {
                    for (MotorDevice motor : MOTORS) {
                        assertEquals(0.0, rig.percent(motor), 1e-9,
                                "motor " + motor.id() + " still driven after cancel at " + cancelTick + ": " + tree);
                    }
                }
            }
            assertEquals(false, rig.scheduler.isRunning(action));
        }
    }

    // ---------------------------------------------------------------- generator

    private static List<MotorDevice> allMotors() {
        return List.of(MOTORS);
    }

    private static Node generate(Random random, int depth, List<MotorDevice> motors) {
        if (depth <= 0 || random.nextInt(4) == 0) return leaf(random, motors);
        switch (random.nextInt(10)) {
            case 0, 1: {
                Node first = generate(random, depth - 1, motors);
                Node second = generate(random, depth - 1, motors);
                if (random.nextBoolean()) return new Then(List.of(first, second));
                return new Then(List.of(first, second, generate(random, depth - 1, motors)));
            }
            case 2, 3, 4: {
                Group.Kind kind = Group.Kind.values()[random.nextInt(3)];
                List<List<MotorDevice>> split = split(random, motors);
                return new Group(kind, generate(random, depth - 1, split.get(0)), generate(random, depth - 1, split.get(1)));
            }
            case 5, 6: {
                List<Node> steps = new ArrayList<>();
                int count = 1 + random.nextInt(3);
                for (int index = 0; index < count; index++) steps.add(step(random, depth - 1, motors));
                return new Sequence(steps);
            }
            case 7:
                return new Timeout(generate(random, depth - 1, motors), random.nextInt(6));
            case 8:
                return new Either(random.nextBoolean(), generate(random, depth - 1, motors),
                        random.nextBoolean() ? generate(random, depth - 1, motors) : null);
            default:
                return new Until(rawLeaf(random, motors), random.nextInt(5),
                        random.nextBoolean() ? generate(random, depth - 1, motors) : null,
                        random.nextBoolean() ? generate(random, depth - 1, motors) : null);
        }
    }

    private static Node step(Random random, int depth, List<MotorDevice> motors) {
        return switch (random.nextInt(5)) {
            case 0 -> new Sequence.ForTime(rawLeaf(random, motors), random.nextInt(5));
            case 1 -> new Wait(random.nextInt(4), true);
            case 2 -> new DoOnce();
            default -> generate(random, depth, motors);
        };
    }

    private static Node leaf(Random random, List<MotorDevice> motors) {
        int choice = random.nextInt(motors.isEmpty() ? 2 : 5);
        return switch (choice) {
            case 0 -> new Wait(random.nextInt(4), false);
            case 1 -> new DoOnce();
            default -> new Out(rawLeaf(random, motors), random.nextInt(5));
        };
    }

    private static Raw rawLeaf(Random random, List<MotorDevice> motors) {
        if (motors.isEmpty()) return null;
        MotorDevice motor = motors.get(random.nextInt(motors.size()));
        return new Raw(motor, (1 + random.nextInt(9)) / 10.0 * (random.nextBoolean() ? 1 : -1));
    }

    private static List<List<MotorDevice>> split(Random random, List<MotorDevice> motors) {
        List<MotorDevice> left = new ArrayList<>();
        List<MotorDevice> right = new ArrayList<>();
        for (MotorDevice motor : motors) (random.nextBoolean() ? left : right).add(motor);
        return List.of(left, right);
    }

    // ---------------------------------------------------------------- model

    record Raw(MotorDevice motor, double value) {
        Action build() {
            return motor.percent(value);
        }

        @Override
        public String toString() {
            return "m" + motor.id() + "=" + value;
        }
    }

    interface Node {
        Action build(Rig rig);

        int duration();

        /** Records outputs for ticks in [start, cutoff) and do-once executions. */
        void expect(int start, int cutoff, Expectation expectation);

        default void collectDoOnces(List<DoOnce> out) {
        }
    }

    record Out(Raw raw, int ticks) implements Node {
        public Action build(Rig rig) {
            if (raw == null) return Actions.waitSeconds(Seconds.of(ticks * DT));
            return raw.build().timeout(Seconds.of(ticks * DT));
        }

        public int duration() {
            return ticks;
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            if (raw == null) return;
            for (int tick = start; tick < Math.min(start + ticks, cutoff); tick++) expectation.put(tick, raw);
        }

        @Override
        public String toString() {
            return raw + ".timeout(" + ticks + ")";
        }
    }

    record Wait(int ticks, boolean sequenceStep) implements Node {
        public Action build(Rig rig) {
            return Actions.waitSeconds(Seconds.of(ticks * DT));
        }

        public int duration() {
            return ticks;
        }

        public void expect(int start, int cutoff, Expectation expectation) {
        }

        @Override
        public String toString() {
            return "wait(" + ticks + ")";
        }
    }

    static final class DoOnce implements Node {
        private int expectedRuns;
        private int expectedTick = -1;

        public Action build(Rig rig) {
            return Actions.doOnce(() -> rig.ran(this));
        }

        public int duration() {
            return 0;
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            if (start < cutoff) {
                expectedRuns++;
                expectedTick = start;
                expectation.doOnces.put(this, start);
            } else if (start > cutoff) {
                expectation.doOnces.put(this, -1);
            }
        }

        @Override
        public void collectDoOnces(List<DoOnce> out) {
            out.add(this);
        }

        @Override
        public String toString() {
            return "doOnce";
        }
    }

    record Then(List<Node> stages) implements Node {
        public Action build(Rig rig) {
            Action action = stages.get(0).build(rig);
            for (int index = 1; index < stages.size(); index++) action = action.then(stages.get(index).build(rig));
            return action;
        }

        public int duration() {
            return stages.stream().mapToInt(Node::duration).sum();
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            int at = start;
            for (Node stage : stages) {
                stage.expect(at, cutoff, expectation);
                at += stage.duration();
            }
        }

        public void collectDoOnces(List<DoOnce> out) {
            stages.forEach(stage -> stage.collectDoOnces(out));
        }

        @Override
        public String toString() {
            StringBuilder text = new StringBuilder(stages.get(0).toString());
            for (int index = 1; index < stages.size(); index++) text.append(".then(").append(stages.get(index)).append(")");
            return text.toString();
        }
    }

    record Group(Kind kind, Node first, Node second) implements Node {
        enum Kind { PARALLEL, RACE, DEADLINE }

        public Action build(Rig rig) {
            Action a = first.build(rig);
            Action b = second.build(rig);
            return switch (kind) {
                case PARALLEL -> Actions.parallel(a, b);
                case RACE -> Actions.race(a, b);
                case DEADLINE -> Actions.deadline(a, b);
            };
        }

        public int duration() {
            return switch (kind) {
                case PARALLEL -> Math.max(first.duration(), second.duration());
                case RACE -> Math.min(first.duration(), second.duration());
                case DEADLINE -> first.duration();
            };
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            int end = Math.min(cutoff, start + duration());
            first.expect(start, end, expectation);
            second.expect(start, end, expectation);
        }

        public void collectDoOnces(List<DoOnce> out) {
            first.collectDoOnces(out);
            second.collectDoOnces(out);
        }

        @Override
        public String toString() {
            return kind.name().toLowerCase() + "(" + first + ", " + second + ")";
        }
    }

    record Timeout(Node child, int ticks) implements Node {
        public Action build(Rig rig) {
            return child.build(rig).timeout(Seconds.of(ticks * DT));
        }

        public int duration() {
            return Math.min(child.duration(), ticks);
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            child.expect(start, Math.min(cutoff, start + ticks), expectation);
        }

        public void collectDoOnces(List<DoOnce> out) {
            child.collectDoOnces(out);
        }

        @Override
        public String toString() {
            return child + ".timeout(" + ticks + ")";
        }
    }

    record Either(boolean condition, Node active, Node inactive) implements Node {
        public Action build(Rig rig) {
            Action a = active.build(rig);
            if (inactive == null) return Actions.onlyIf(() -> condition, a);
            return Actions.either(() -> condition, a, inactive.build(rig));
        }

        private Node chosen() {
            return condition ? active : inactive;
        }

        public int duration() {
            return chosen() == null ? 0 : chosen().duration();
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            if (chosen() != null) chosen().expect(start, cutoff, expectation);
            Node other = condition ? inactive : active;
            if (other != null) {
                List<DoOnce> skipped = new ArrayList<>();
                other.collectDoOnces(skipped);
                skipped.forEach(doOnce -> expectation.doOnces.putIfAbsent(doOnce, -1));
            }
        }

        public void collectDoOnces(List<DoOnce> out) {
            active.collectDoOnces(out);
            if (inactive != null) inactive.collectDoOnces(out);
        }

        @Override
        public String toString() {
            return (inactive == null ? "onlyIf(" + condition + ", " + active : "either(" + condition + ", " + active + ", " + inactive) + ")";
        }
    }

    /** raw.until(time >= ticks) with optional then() stages, exercising until(...).then(...).then(...). */
    record Until(Raw raw, int ticks, Node next, Node after) implements Node {
        public Action build(Rig rig) {
            Action base = raw == null ? Actions.waitUntil(() -> false) : raw.build();
            Action action = base.until(ctx -> ctx.timeInStateSeconds() + 1e-9 >= ticks * DT);
            if (next != null) action = action.then(next.build(rig));
            if (after != null) action = action.then(after.build(rig));
            return action;
        }

        public int duration() {
            return ticks + (next == null ? 0 : next.duration()) + (after == null ? 0 : after.duration());
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            if (raw != null) {
                for (int tick = start; tick < Math.min(start + ticks, cutoff); tick++) expectation.put(tick, raw);
            }
            int at = start + ticks;
            if (next != null) {
                next.expect(at, cutoff, expectation);
                at += next.duration();
            }
            if (after != null) after.expect(at, cutoff, expectation);
        }

        public void collectDoOnces(List<DoOnce> out) {
            if (next != null) next.collectDoOnces(out);
            if (after != null) after.collectDoOnces(out);
        }

        @Override
        public String toString() {
            return (raw == null ? "neutral" : raw.toString()) + ".until(" + ticks + ")"
                    + (next == null ? "" : ".then(" + next + ")") + (after == null ? "" : ".then(" + after + ")");
        }
    }

    record Sequence(List<Node> steps) implements Node {
        record ForTime(Raw raw, int ticks) implements Node {
            public Action build(Rig rig) {
                throw new UnsupportedOperationException("sequence-only step");
            }

            public int duration() {
                return ticks;
            }

            public void expect(int start, int cutoff, Expectation expectation) {
                if (raw == null) return;
                for (int tick = start; tick < Math.min(start + ticks, cutoff); tick++) expectation.put(tick, raw);
            }

            @Override
            public String toString() {
                return "forTime(" + ticks + ", " + raw + ")";
            }
        }

        public Action build(Rig rig) {
            Actions.Sequence sequence = Actions.sequence();
            for (Node step : steps) {
                if (step instanceof ForTime forTime) {
                    sequence = sequence.forTime(Seconds.of(forTime.ticks * DT),
                            forTime.raw == null ? Actions.waitUntil(() -> false) : forTime.raw.build());
                } else if (step instanceof Wait wait && wait.sequenceStep()) {
                    sequence = sequence.waitSeconds(Seconds.of(wait.ticks() * DT));
                } else if (step instanceof DoOnce doOnce) {
                    sequence = sequence.doOnce(() -> rig.ran(doOnce));
                } else {
                    sequence = sequence.run(step.build(rig));
                }
            }
            return sequence;
        }

        public int duration() {
            return steps.stream().mapToInt(Node::duration).sum();
        }

        public void expect(int start, int cutoff, Expectation expectation) {
            int at = start;
            for (Node step : steps) {
                step.expect(at, cutoff, expectation);
                at += step.duration();
            }
        }

        public void collectDoOnces(List<DoOnce> out) {
            steps.forEach(step -> step.collectDoOnces(out));
        }

        @Override
        public String toString() {
            StringBuilder text = new StringBuilder("sequence(");
            for (int index = 0; index < steps.size(); index++) text.append(index == 0 ? "" : ", ").append(steps.get(index));
            return text.append(")").toString();
        }
    }

    static final class Expectation {
        private final Map<Integer, Map<MotorDevice, Double>> outputs = new HashMap<>();
        private final Map<DoOnce, Integer> doOnces = new IdentityHashMap<>();

        void put(int tick, Raw raw) {
            Double previous = outputs.computeIfAbsent(tick, ignored -> new HashMap<>()).put(raw.motor(), raw.value());
            if (previous != null) throw new IllegalStateException("generator produced overlapping outputs");
        }

        Double value(int tick, MotorDevice motor) {
            Map<MotorDevice, Double> values = outputs.get(tick);
            return values == null ? null : values.get(motor);
        }

        void assertTick(Rig rig, int tick, Node tree, String note) {
            for (MotorDevice motor : MOTORS) {
                Double want = value(tick, motor);
                double actual = rig.percent(motor);
                if (Math.abs(actual - (want == null ? 0.0 : want)) > 1e-9) {
                    fail("tick " + tick + " motor " + motor.id() + " expected " + (want == null ? "neutral" : want)
                            + " but was " + actual + (note == null ? "" : " (" + note + ")") + "\n tree=" + tree);
                }
            }
        }

        void assertDoOnces(Node tree) {
            doOnces.forEach((doOnce, tick) -> {
                Rig.Run run = Rig.runs.get(doOnce);
                if (tick < 0) {
                    if (run != null) fail("doOnce ran but its branch was skipped/cut off: " + tree);
                } else if (run == null || run.count != 1 || run.tick != tick) {
                    fail("doOnce expected once at tick " + tick + " but got "
                            + (run == null ? "none" : run.count + " runs, last at " + run.tick) + ": " + tree);
                }
            });
        }
    }

    // ---------------------------------------------------------------- runtime rig

    static final class Rig implements Mechanism {
        static final Map<DoOnce, Run> runs = new IdentityHashMap<>();
        private final MotorDevice m0 = MOTORS[0];
        private final MotorDevice m1 = MOTORS[1];
        private final MotorDevice m2 = MOTORS[2];
        private final MotorDevice m3 = MOTORS[3];
        final RecordingContext context = new RecordingContext();
        final MechanismScheduler scheduler;
        final Map<DoOnce, Run> counters = runs;
        private int currentTick;

        static final class Run {
            int count;
            int tick;
        }

        Rig() {
            runs.clear();
            scheduler = MechanismScheduler.create(context).register(this);
        }

        void tick(int tick) {
            currentTick = tick;
            scheduler.teleopPeriodic(tick * DT, DT);
        }

        void ran(DoOnce doOnce) {
            Run run = runs.computeIfAbsent(doOnce, ignored -> new Run());
            run.count++;
            run.tick = currentTick;
        }

        double percent(MotorDevice motor) {
            double value = context.motor(motor).percent;
            return Double.isNaN(value) ? 0.0 : value;
        }
    }

    static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new IdentityHashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        double percent = Double.NaN;

        RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
        }

        @Override
        public void stop() {
            percent = 0.0;
        }
    }
}
