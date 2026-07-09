package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Factories for common mechanism states.
 */
public final class States {
    private States() {
    }

    public static State neutral() {
        return new Neutral();
    }

    public static State percent(MotorDevice motor, double percent) {
        return new MotorPercent(motor, percent);
    }

    public static State percent(MotorDevice motor, DoubleSupplier percent) {
        return new DynamicMotorPercent(motor, percent);
    }

    public static State percent(ControlBinding control, double percent) {
        return new ControlPercent(control, percent);
    }

    public static State percent(ControlBinding control, DoubleSupplier percent) {
        return new DynamicControlPercent(control, percent);
    }

    public static State voltage(MotorDevice motor, double volts) {
        return new MotorVoltage(motor, volts);
    }

    public static State voltage(MotorDevice motor, DoubleSupplier volts) {
        return new DynamicMotorVoltage(motor, volts);
    }

    public static State voltage(ControlBinding control, double volts) {
        return new ControlVoltage(control, volts);
    }

    public static State voltage(ControlBinding control, DoubleSupplier volts) {
        return new DynamicControlVoltage(control, volts);
    }

    public static State position(ControlBinding control, double position) {
        return new ControlPosition(control, position);
    }

    public static State position(ControlBinding control, DoubleSupplier position) {
        return new DynamicControlPosition(control, position);
    }

    public static State velocity(ControlBinding control, double velocity) {
        return new ControlVelocity(control, velocity);
    }

    public static State velocity(ControlBinding control, DoubleSupplier velocity) {
        return new DynamicControlVelocity(control, velocity);
    }

    public static State fault(String reason) {
        return new Fault(reason);
    }

    public static ChildSet set() {
        return new ChildSet();
    }

    public static ChildSet from(ChildSet state) {
        Objects.requireNonNull(state, "state");
        ChildSet copy = set();
        for (ChildTarget target : state.targets()) {
            copy.set(target.mechanism(), target.state());
        }
        return copy;
    }

    public static State dynamic(Supplier<Output> output) {
        Objects.requireNonNull(output, "output");
        return dynamic(ctx -> output.get());
    }

    public static State dynamic(Function<MechanismContext, Output> output) {
        return new DynamicOutput(output);
    }

    public static State doOnce(Runnable action) {
        return new DoOnce(action);
    }

    public static State doOnce(ActionBinding action) {
        return new Action(action);
    }

    public static State run(Runnable action) {
        return doOnce(action);
    }

    public static State run(ActionBinding action) {
        return doOnce(action);
    }

    public static State waitSeconds(double seconds) {
        return new WaitSeconds(seconds);
    }

    public static State waitUntil(StateCondition condition) {
        return new WaitUntil(condition);
    }

    public static State waitUntil(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return waitUntil(ctx -> condition.getAsBoolean());
    }

    public static State timeout(State state, double seconds) {
        return new Timeout(state, seconds);
    }

    public static When when(StateCondition condition) {
        return new When(condition);
    }

    public static When when(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return when(ctx -> condition.getAsBoolean());
    }

    public static Parallel parallel(State... states) {
        return new Parallel(List.of(states));
    }

    public static Race race(State... states) {
        return new Race(List.of(states));
    }

    public static Deadline deadline(State primary, State... others) {
        return new Deadline(primary, List.of(others));
    }

    public static Sequence sequence() {
        return new Sequence();
    }

    public static Cycle cycle() {
        return new Cycle();
    }

    public static State until(StateCondition condition, State state) {
        return new Conditional(state, condition, null);
    }

    public static State until(BooleanSupplier condition, State state) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean(), state);
    }

    public static State then(State state, State next) {
        Objects.requireNonNull(next, "next");
        if (state instanceof State.Conditional conditional) {
            return new State.Conditional(conditional.state(), conditional.condition(), next);
        }
        if (state instanceof Conditional conditional) {
            return new Conditional(conditional.state(), conditional.condition(), next);
        }
        if (state instanceof Sequence sequence) {
            return sequence.then(next);
        }
        return new Then(state, next);
    }

    public static State clamp(State state, Range range) {
        return new Clamped(state, range);
    }

    public record Neutral() implements State, Output.Neutral {
    }

    public record MotorPercent(MotorDevice motor, double percent) implements State, Output.Percent {
        public MotorPercent {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record DynamicMotorPercent(MotorDevice motor, DoubleSupplier percentSupplier) implements State, Output.Percent {
        public DynamicMotorPercent {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    public record MotorVoltage(MotorDevice motor, double volts) implements State, Output.Voltage {
        public MotorVoltage {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record DynamicMotorVoltage(MotorDevice motor, DoubleSupplier voltsSupplier) implements State, Output.Voltage {
        public DynamicMotorVoltage {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }

    public record ControlPercent(ControlBinding control, double percent) implements State, Output.Percent {
        public ControlPercent {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlPercent(ControlBinding control, DoubleSupplier percentSupplier) implements State, Output.Percent {
        public DynamicControlPercent {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    public record ControlVoltage(ControlBinding control, double volts) implements State, Output.Voltage {
        public ControlVoltage {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlVoltage(ControlBinding control, DoubleSupplier voltsSupplier) implements State, Output.Voltage {
        public DynamicControlVoltage {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }

    public record ControlPosition(ControlBinding control, double position) implements State, Output.Position {
        public ControlPosition {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlPosition(ControlBinding control, DoubleSupplier positionSupplier) implements State, Output.Position {
        public DynamicControlPosition {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(positionSupplier, "positionSupplier");
        }

        @Override
        public double position() {
            return positionSupplier.getAsDouble();
        }
    }

    public record ControlVelocity(ControlBinding control, double velocity) implements State, Output.Velocity {
        public ControlVelocity {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlVelocity(ControlBinding control, DoubleSupplier velocitySupplier) implements State, Output.Velocity {
        public DynamicControlVelocity {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(velocitySupplier, "velocitySupplier");
        }

        @Override
        public double velocity() {
            return velocitySupplier.getAsDouble();
        }
    }

    public record Fault(String reason) implements State, Output.Fault {
        public Fault {
            reason = reason == null ? "" : reason;
        }
    }

    public record DoOnce(Runnable action) implements State {
        public DoOnce {
            Objects.requireNonNull(action, "action");
        }
    }

    public record Action(ActionBinding action) implements State {
        public Action {
            Objects.requireNonNull(action, "action");
        }
    }

    public record WaitSeconds(double seconds) implements State {
        public boolean complete(MechanismContext ctx) {
            return ctx.timeInStateSeconds() >= seconds;
        }
    }

    public record WaitUntil(StateCondition condition) implements State {
        public WaitUntil {
            Objects.requireNonNull(condition, "condition");
        }

        public boolean complete(MechanismContext ctx) {
            return condition.test(ctx);
        }
    }

    public record Timeout(State state, double seconds) implements State {
        public Timeout {
            Objects.requireNonNull(state, "state");
        }

        public boolean expired(MechanismContext ctx) {
            return ctx.timeInStateSeconds() >= seconds;
        }
    }

    public record Choice(StateCondition condition, State active, State inactive) implements State {
        public Choice {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
            Objects.requireNonNull(inactive, "inactive");
        }

        public State choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : inactive;
        }
    }

    public record WhenBranch(StateCondition condition, State active) implements State {
        public WhenBranch {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
        }

        public Choice otherwise(State inactive) {
            return new Choice(condition, active, inactive);
        }

        public State choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : neutral();
        }
    }

    public record Parallel(List<State> states) implements State {
        public Parallel {
            states = copyStates(states);
        }
    }

    public record Race(List<State> states) implements State {
        public Race {
            states = copyStates(states);
        }
    }

    public record Deadline(State primary, List<State> others) implements State {
        public Deadline {
            Objects.requireNonNull(primary, "primary");
            others = copyStates(others);
        }

        public List<State> states() {
            List<State> states = new ArrayList<>();
            states.add(primary);
            states.addAll(others);
            return List.copyOf(states);
        }
    }

    public record DynamicOutput(Function<MechanismContext, Output> output) implements State {
        public DynamicOutput {
            Objects.requireNonNull(output, "output");
        }
    }

    public record Conditional(State state, StateCondition condition, State next) implements State {
        public Conditional {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(condition, "condition");
        }
    }

    public record Then(State state, State next) implements State {
        public Then {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(next, "next");
        }
    }

    public record Clamped(State state, Range range) implements State {
        public Clamped {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(range, "range");
        }
    }

    public static final class ChildSet implements State {
        private final List<ChildTarget> targets = new ArrayList<>();

        public ChildSet set(Mechanism mechanism, State state) {
            targets.add(new ChildTarget(mechanism, state));
            return this;
        }

        public List<ChildTarget> targets() {
            return List.copyOf(targets);
        }
    }

    public record ChildTarget(Mechanism mechanism, State state) {
        public ChildTarget {
            Objects.requireNonNull(mechanism, "mechanism");
            Objects.requireNonNull(state, "state");
        }
    }

    public static final class Sequence implements State {
        private final List<SequenceStep> steps = new ArrayList<>();
        private State next;
        private double timeoutSeconds = Double.POSITIVE_INFINITY;

        private Sequence() {
        }

        public Sequence run(State state) {
            steps.add(new SequenceStep(state, ctx -> false));
            return this;
        }

        public Sequence forTime(double seconds, State state) {
            steps.add(new SequenceStep(state, ctx -> ctx.timeInStateSeconds() >= seconds));
            return this;
        }

        public Sequence until(StateCondition condition, State state) {
            steps.add(new SequenceStep(state, condition));
            return this;
        }

        public Sequence until(BooleanSupplier condition, State state) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), state);
        }

        public Sequence doOnce(Runnable action) {
            steps.add(new SequenceStep(States.doOnce(action), ctx -> true));
            return this;
        }

        public Sequence doOnce(ActionBinding action) {
            steps.add(new SequenceStep(States.doOnce(action), ctx -> true));
            return this;
        }

        public Sequence waitSeconds(double seconds) {
            steps.add(new SequenceStep(States.waitSeconds(seconds), ctx -> ctx.timeInStateSeconds() >= seconds));
            return this;
        }

        public Sequence waitUntil(StateCondition condition) {
            steps.add(new SequenceStep(States.waitUntil(condition), condition));
            return this;
        }

        public Sequence waitUntil(BooleanSupplier condition) {
            Objects.requireNonNull(condition, "condition");
            return waitUntil(ctx -> condition.getAsBoolean());
        }

        public Sequence timeout(double seconds) {
            timeoutSeconds = seconds;
            return this;
        }

        @Override
        public Sequence then(State state) {
            next = Objects.requireNonNull(state, "state");
            return this;
        }

        public List<SequenceStep> steps() {
            return List.copyOf(steps);
        }

        public State next() {
            return next;
        }

        public double timeoutSeconds() {
            return timeoutSeconds;
        }
    }

    public static final class When {
        private final StateCondition condition;

        private When(StateCondition condition) {
            this.condition = Objects.requireNonNull(condition, "condition");
        }

        public WhenBranch run(State state) {
            return new WhenBranch(condition, state);
        }

        public WhenBranch then(State state) {
            return run(state);
        }
    }

    public static final class Cycle implements State {
        private final List<CycleStep> steps = new ArrayList<>();

        private Cycle() {
        }

        public Cycle forTime(double seconds, State state) {
            steps.add(new CycleStep(state, ctx -> ctx.timeInStateSeconds() >= seconds));
            return this;
        }

        public Cycle until(StateCondition condition, State state) {
            steps.add(new CycleStep(state, condition));
            return this;
        }

        public Cycle until(BooleanSupplier condition, State state) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), state);
        }

        public List<CycleStep> steps() {
            return List.copyOf(steps);
        }
    }

    public record CycleStep(State state, StateCondition advance) {
        public CycleStep {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(advance, "advance");
        }
    }

    public record SequenceStep(State state, StateCondition complete) {
        public SequenceStep {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(complete, "complete");
        }
    }

    private static List<State> copyStates(List<State> states) {
        Objects.requireNonNull(states, "states");
        List<State> copy = new ArrayList<>();
        for (State state : states) {
            copy.add(Objects.requireNonNull(state, "state"));
        }
        return List.copyOf(copy);
    }
}
