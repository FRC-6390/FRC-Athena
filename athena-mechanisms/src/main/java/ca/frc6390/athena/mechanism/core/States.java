package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
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

    /**
     * Creates a neutral state.
     *
     * @return neutral state
     */
    public static MechanismState neutral() {
        return new Neutral();
    }

    /**
     * Creates a percent-output state for a specific motor.
     *
     * @param motor motor target
     * @param percent percent output
     * @return targeted percent state
     */
    public static MechanismState percent(MotorRef motor, double percent) {
        return new MotorPercent(motor, percent);
    }

    /**
     * Creates a dynamic percent-output state for a specific motor.
     *
     * @param motor motor target
     * @param percent percent supplier
     * @return dynamic motor percent state
     */
    public static MechanismState percent(MotorRef motor, DoubleSupplier percent) {
        return new DynamicMotorPercent(motor, percent);
    }

    /**
     * Creates a percent-output state for a specific axis binding.
     *
     * @param axis axis binding
     * @param percent percent output
     * @return axis percent state
     */
    public static MechanismState percent(AxisRef axis, double percent) {
        return new AxisPercent(axis, percent);
    }

    /**
     * Creates a dynamic percent-output state for a specific axis binding.
     *
     * @param axis axis binding
     * @param percent percent supplier
     * @return dynamic axis percent state
     */
    public static MechanismState percent(AxisRef axis, DoubleSupplier percent) {
        return new DynamicAxisPercent(axis, percent);
    }

    public static MechanismState percent(ControlRef control, double percent) {
        return new ControlPercent(control, percent);
    }

    public static MechanismState percent(ControlRef control, DoubleSupplier percent) {
        return new DynamicControlPercent(control, percent);
    }

    public static MechanismState voltage(MotorRef motor, double volts) {
        return new MotorVoltage(motor, volts);
    }

    public static MechanismState voltage(MotorRef motor, DoubleSupplier volts) {
        return new DynamicMotorVoltage(motor, volts);
    }

    public static MechanismState voltage(AxisRef axis, double volts) {
        return new AxisVoltage(axis, volts);
    }

    public static MechanismState voltage(AxisRef axis, DoubleSupplier volts) {
        return new DynamicAxisVoltage(axis, volts);
    }

    public static MechanismState voltage(ControlRef control, double volts) {
        return new ControlVoltage(control, volts);
    }

    public static MechanismState voltage(ControlRef control, DoubleSupplier volts) {
        return new DynamicControlVoltage(control, volts);
    }

    /**
     * Creates a position state for a specific axis binding.
     *
     * @param axis axis binding
     * @param position position target
     * @return axis position state
     */
    public static MechanismState position(AxisRef axis, double position) {
        return new AxisPosition(axis, position);
    }

    /**
     * Creates a dynamic position state for a specific axis binding.
     *
     * @param axis axis binding
     * @param position position supplier
     * @return dynamic axis position state
     */
    public static MechanismState position(AxisRef axis, DoubleSupplier position) {
        return new DynamicAxisPosition(axis, position);
    }

    public static MechanismState position(ControlRef control, double position) {
        return new ControlPosition(control, position);
    }

    public static MechanismState position(ControlRef control, DoubleSupplier position) {
        return new DynamicControlPosition(control, position);
    }

    /**
     * Creates a velocity state for a specific axis binding.
     *
     * @param axis axis binding
     * @param velocity velocity target
     * @return axis velocity state
     */
    public static MechanismState velocity(AxisRef axis, double velocity) {
        return new AxisVelocity(axis, velocity);
    }

    /**
     * Creates a dynamic velocity state for a specific axis binding.
     *
     * @param axis axis binding
     * @param velocity velocity supplier
     * @return dynamic axis velocity state
     */
    public static MechanismState velocity(AxisRef axis, DoubleSupplier velocity) {
        return new DynamicAxisVelocity(axis, velocity);
    }

    public static MechanismState velocity(ControlRef control, double velocity) {
        return new ControlVelocity(control, velocity);
    }

    public static MechanismState velocity(ControlRef control, DoubleSupplier velocity) {
        return new DynamicControlVelocity(control, velocity);
    }

    /**
     * Creates a fault state.
     *
     * @param reason fault reason
     * @return fault state
     */
    public static MechanismState fault(String reason) {
        return new Fault(reason);
    }

    /**
     * Creates a child-target state.
     *
     * @return child set
     */
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

    /**
     * Creates a dynamic state that computes an output at runtime.
     *
     * @param output output supplier
     * @return dynamic output state
     */
    public static MechanismState dynamic(Supplier<Output> output) {
        Objects.requireNonNull(output, "output");
        return dynamic(ctx -> output.get());
    }

    /**
     * Creates a dynamic state that computes an output from lifecycle context.
     *
     * @param output output function
     * @return dynamic output state
     */
    public static MechanismState dynamic(Function<MechanismContext, Output> output) {
        return new DynamicOutput(output);
    }

    /**
     * Creates a one-shot action state.
     *
     * @param action action to run once when the state is entered
     * @return action state
     */
    public static MechanismState doOnce(Runnable action) {
        return new DoOnce(action);
    }

    /**
     * Creates a one-shot action state that can access runtime refs.
     *
     * @param action action to run once when the state is entered
     * @return action state
     */
    public static MechanismState doOnce(ActionRef action) {
        return new Action(action);
    }

    /**
     * Creates a one-shot action state.
     *
     * @param action action to run once when the state is entered
     * @return action state
     */
    public static MechanismState run(Runnable action) {
        return doOnce(action);
    }

    /**
     * Creates a one-shot action state that can access runtime refs.
     *
     * @param action action to run once when the state is entered
     * @return action state
     */
    public static MechanismState run(ActionRef action) {
        return doOnce(action);
    }

    /**
     * Creates a wait state that completes after a fixed duration.
     *
     * @param seconds wait duration
     * @return wait state
     */
    public static MechanismState waitSeconds(double seconds) {
        return new WaitSeconds(seconds);
    }

    /**
     * Creates a wait state that completes when a condition is true.
     *
     * @param condition completion condition
     * @return wait state
     */
    public static MechanismState waitUntil(StateCondition condition) {
        return new WaitUntil(condition);
    }

    /**
     * Creates a wait state that completes when a no-argument condition is true.
     *
     * @param condition completion condition
     * @return wait state
     */
    public static MechanismState waitUntil(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return waitUntil(ctx -> condition.getAsBoolean());
    }

    /**
     * Caps how long a state can remain active.
     *
     * @param state wrapped state
     * @param seconds timeout duration
     * @return timeout-constrained state
     */
    public static MechanismState timeout(MechanismState state, double seconds) {
        return new Timeout(state, seconds);
    }

    /**
     * Starts a conditional state builder.
     *
     * @param condition active condition
     * @return conditional builder
     */
    public static When when(StateCondition condition) {
        return new When(condition);
    }

    /**
     * Starts a conditional state builder.
     *
     * @param condition active condition
     * @return conditional builder
     */
    public static When when(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return when(ctx -> condition.getAsBoolean());
    }

    /**
     * Runs states together.
     *
     * @param states states to run
     * @return parallel state
     */
    public static Parallel parallel(MechanismState... states) {
        return new Parallel(List.of(states));
    }

    /**
     * Runs states together until any child completes.
     *
     * @param states states to race
     * @return race state
     */
    public static Race race(MechanismState... states) {
        return new Race(List.of(states));
    }

    /**
     * Runs secondary states until the primary state completes.
     *
     * @param primary primary state
     * @param others secondary states
     * @return deadline state
     */
    public static Deadline deadline(MechanismState primary, MechanismState... others) {
        return new Deadline(primary, List.of(others));
    }

    /**
     * Starts a state sequence.
     *
     * @return sequence builder
     */
    public static Sequence sequence() {
        return new Sequence();
    }

    /**
     * Starts a repeating state cycle.
     *
     * @return cycle builder
     */
    public static Cycle cycle() {
        return new Cycle();
    }

    /**
     * Wraps a state in an until condition.
     *
     * @param condition condition
     * @param state wrapped state
     * @return conditional state
     */
    public static MechanismState until(StateCondition condition, MechanismState state) {
        return new Conditional(state, condition, null);
    }

    /**
     * Wraps a state in a no-argument until condition.
     *
     * @param condition condition
     * @param state wrapped state
     * @return conditional state
     */
    public static MechanismState until(BooleanSupplier condition, MechanismState state) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean(), state);
    }

    /**
     * Adds a transition target.
     *
     * @param state source state
     * @param next next state
     * @return state with transition target
     */
    public static MechanismState then(MechanismState state, MechanismState next) {
        Objects.requireNonNull(next, "next");
        if (state instanceof MechanismState.Conditional conditional) {
            return new MechanismState.Conditional(conditional.state(), conditional.condition(), next);
        }
        if (state instanceof Conditional conditional) {
            return new Conditional(conditional.state(), conditional.condition(), next);
        }
        if (state instanceof Sequence sequence) {
            return sequence.then(next);
        }
        return new Then(state, next);
    }

    /**
     * Adds a range constraint to a state.
     *
     * @param state state
     * @param range range
     * @return constrained state
     */
    public static MechanismState clamp(MechanismState state, RangeRef range) {
        return new Clamped(state, range);
    }

    public record Neutral() implements MechanismState, Output.Neutral {
    }

    public record MotorPercent(MotorRef motor, double percent) implements MechanismState, Output.Percent {
        public MotorPercent {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record DynamicMotorPercent(MotorRef motor, DoubleSupplier percentSupplier)
            implements MechanismState, Output.Percent {
        public DynamicMotorPercent {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    public record MotorVoltage(MotorRef motor, double volts) implements MechanismState, Output.Voltage {
        public MotorVoltage {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record DynamicMotorVoltage(MotorRef motor, DoubleSupplier voltsSupplier)
            implements MechanismState, Output.Voltage {
        public DynamicMotorVoltage {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }

    public record AxisPercent(AxisRef axis, double percent) implements MechanismState, Output.Percent {
        public AxisPercent {
            Objects.requireNonNull(axis, "axis");
        }
    }

    public record DynamicAxisPercent(AxisRef axis, DoubleSupplier percentSupplier)
            implements MechanismState, Output.Percent {
        public DynamicAxisPercent {
            Objects.requireNonNull(axis, "axis");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    public record AxisVoltage(AxisRef axis, double volts) implements MechanismState, Output.Voltage {
        public AxisVoltage {
            Objects.requireNonNull(axis, "axis");
        }
    }

    public record DynamicAxisVoltage(AxisRef axis, DoubleSupplier voltsSupplier)
            implements MechanismState, Output.Voltage {
        public DynamicAxisVoltage {
            Objects.requireNonNull(axis, "axis");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }

    public record ControlPercent(ControlRef control, double percent)
            implements MechanismState, Output.Percent {
        public ControlPercent {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlPercent(ControlRef control, DoubleSupplier percentSupplier)
            implements MechanismState, Output.Percent {
        public DynamicControlPercent {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    public record ControlVoltage(ControlRef control, double volts)
            implements MechanismState, Output.Voltage {
        public ControlVoltage {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlVoltage(ControlRef control, DoubleSupplier voltsSupplier)
            implements MechanismState, Output.Voltage {
        public DynamicControlVoltage {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }

    public record AxisPosition(AxisRef axis, double position)
            implements MechanismState, Output.Position {
        public AxisPosition {
            Objects.requireNonNull(axis, "axis");
        }
    }

    public record ControlPosition(ControlRef control, double position)
            implements MechanismState, Output.Position {
        public ControlPosition {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlPosition(ControlRef control, DoubleSupplier positionSupplier)
            implements MechanismState, Output.Position {
        public DynamicControlPosition {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(positionSupplier, "positionSupplier");
        }

        @Override
        public double position() {
            return positionSupplier.getAsDouble();
        }
    }

    public record DynamicAxisPosition(AxisRef axis, DoubleSupplier positionSupplier)
            implements MechanismState, Output.Position {
        public DynamicAxisPosition {
            Objects.requireNonNull(axis, "axis");
            Objects.requireNonNull(positionSupplier, "positionSupplier");
        }

        @Override
        public double position() {
            return positionSupplier.getAsDouble();
        }
    }

    public record AxisVelocity(AxisRef axis, double velocity)
            implements MechanismState, Output.Velocity {
        public AxisVelocity {
            Objects.requireNonNull(axis, "axis");
        }
    }

    public record DynamicAxisVelocity(AxisRef axis, DoubleSupplier velocitySupplier)
            implements MechanismState, Output.Velocity {
        public DynamicAxisVelocity {
            Objects.requireNonNull(axis, "axis");
            Objects.requireNonNull(velocitySupplier, "velocitySupplier");
        }

        @Override
        public double velocity() {
            return velocitySupplier.getAsDouble();
        }
    }

    public record ControlVelocity(ControlRef control, double velocity)
            implements MechanismState, Output.Velocity {
        public ControlVelocity {
            Objects.requireNonNull(control, "control");
        }
    }

    public record DynamicControlVelocity(ControlRef control, DoubleSupplier velocitySupplier)
            implements MechanismState, Output.Velocity {
        public DynamicControlVelocity {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(velocitySupplier, "velocitySupplier");
        }

        @Override
        public double velocity() {
            return velocitySupplier.getAsDouble();
        }
    }

    public record Fault(String reason) implements MechanismState, Output.Fault {
        public Fault {
            reason = reason == null ? "" : reason;
        }
    }

    public record DoOnce(Runnable action) implements MechanismState {
        public DoOnce {
            Objects.requireNonNull(action, "action");
        }
    }

    public record Action(ActionRef action) implements MechanismState {
        public Action {
            Objects.requireNonNull(action, "action");
        }
    }

    public record WaitSeconds(double seconds) implements MechanismState {
        public boolean complete(MechanismContext ctx) {
            return ctx.timeInStateSeconds() >= seconds;
        }
    }

    public record WaitUntil(StateCondition condition) implements MechanismState {
        public WaitUntil {
            Objects.requireNonNull(condition, "condition");
        }

        public boolean complete(MechanismContext ctx) {
            return condition.test(ctx);
        }
    }

    public record Timeout(MechanismState state, double seconds) implements MechanismState {
        public Timeout {
            Objects.requireNonNull(state, "state");
        }

        public boolean expired(MechanismContext ctx) {
            return ctx.timeInStateSeconds() >= seconds;
        }
    }

    public record Choice(
            StateCondition condition,
            MechanismState active,
            MechanismState inactive) implements MechanismState {
        public Choice {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
            Objects.requireNonNull(inactive, "inactive");
        }

        public MechanismState choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : inactive;
        }
    }

    public record WhenBranch(StateCondition condition, MechanismState active) implements MechanismState {
        public WhenBranch {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
        }

        public Choice otherwise(MechanismState inactive) {
            return new Choice(condition, active, inactive);
        }

        public MechanismState choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : neutral();
        }
    }

    public record Parallel(List<MechanismState> states) implements MechanismState {
        public Parallel {
            states = copyStates(states);
        }
    }

    public record Race(List<MechanismState> states) implements MechanismState {
        public Race {
            states = copyStates(states);
        }
    }

    public record Deadline(MechanismState primary, List<MechanismState> others) implements MechanismState {
        public Deadline {
            Objects.requireNonNull(primary, "primary");
            others = copyStates(others);
        }

        public List<MechanismState> states() {
            List<MechanismState> states = new ArrayList<>();
            states.add(primary);
            states.addAll(others);
            return List.copyOf(states);
        }
    }

    public record DynamicOutput(Function<MechanismContext, Output> output) implements MechanismState {
        public DynamicOutput {
            Objects.requireNonNull(output, "output");
        }
    }

    public record Conditional(
            MechanismState state,
            StateCondition condition,
            MechanismState next) implements MechanismState {
        public Conditional {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(condition, "condition");
        }
    }

    public record Then(MechanismState state, MechanismState next) implements MechanismState {
        public Then {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(next, "next");
        }
    }

    public record Clamped(MechanismState state, RangeRef range) implements MechanismState {
        public Clamped {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(range, "range");
        }
    }

    public static final class ChildSet implements MechanismState {
        private final List<ChildTarget> targets = new ArrayList<>();

        /**
         * Adds a child state target.
         *
         * @param mechanism child mechanism
         * @param state child target state
         * @return this state
         */
        public ChildSet set(Mechanism mechanism, MechanismState state) {
            targets.add(new ChildTarget(mechanism, state));
            return this;
        }

        /**
         * Returns child targets.
         *
         * @return child targets
         */
        public List<ChildTarget> targets() {
            return List.copyOf(targets);
        }
    }

    public record ChildTarget(Mechanism mechanism, MechanismState state) {
        public ChildTarget {
            Objects.requireNonNull(mechanism, "mechanism");
            Objects.requireNonNull(state, "state");
        }
    }

    public static final class Sequence implements MechanismState {
        private final List<SequenceStep> steps;
        private MechanismState next;
        private double timeoutSeconds = Double.POSITIVE_INFINITY;

        private Sequence() {
            this.steps = new ArrayList<>();
        }

        public Sequence run(MechanismState state) {
            steps.add(new SequenceStep(state, ctx -> false));
            return this;
        }

        /**
         * Adds a step that completes after a fixed duration.
         *
         * @param seconds step duration
         * @param state step state
         * @return this sequence
         */
        public Sequence forTime(double seconds, MechanismState state) {
            steps.add(new SequenceStep(state, ctx -> ctx.timeInStateSeconds() >= seconds));
            return this;
        }

        /**
         * Adds a step that completes when a condition is true.
         *
         * @param condition completion condition
         * @param state step state
         * @return this sequence
         */
        public Sequence until(StateCondition condition, MechanismState state) {
            steps.add(new SequenceStep(state, condition));
            return this;
        }

        /**
         * Adds a step that completes when a no-argument condition is true.
         *
         * @param condition completion condition
         * @param state step state
         * @return this sequence
         */
        public Sequence until(BooleanSupplier condition, MechanismState state) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), state);
        }

        /**
         * Adds a one-shot action step.
         *
         * @param action action to run once
         * @return this sequence
         */
        public Sequence doOnce(Runnable action) {
            steps.add(new SequenceStep(States.doOnce(action), ctx -> true));
            return this;
        }

        /**
         * Adds a one-shot action step that can access runtime refs.
         *
         * @param action action to run once
         * @return this sequence
         */
        public Sequence doOnce(ActionRef action) {
            steps.add(new SequenceStep(States.doOnce(action), ctx -> true));
            return this;
        }

        /**
         * Adds a wait step that completes after a fixed duration.
         *
         * @param seconds wait duration
         * @return this sequence
         */
        public Sequence waitSeconds(double seconds) {
            steps.add(new SequenceStep(States.waitSeconds(seconds), ctx -> ctx.timeInStateSeconds() >= seconds));
            return this;
        }

        /**
         * Adds a wait step that completes when a condition is true.
         *
         * @param condition completion condition
         * @return this sequence
         */
        public Sequence waitUntil(StateCondition condition) {
            steps.add(new SequenceStep(States.waitUntil(condition), condition));
            return this;
        }

        /**
         * Adds a wait step that completes when a no-argument condition is true.
         *
         * @param condition completion condition
         * @return this sequence
         */
        public Sequence waitUntil(BooleanSupplier condition) {
            Objects.requireNonNull(condition, "condition");
            return waitUntil(ctx -> condition.getAsBoolean());
        }

        /**
         * Caps the entire sequence duration.
         *
         * @param seconds timeout duration
         * @return this sequence
         */
        public Sequence timeout(double seconds) {
            timeoutSeconds = seconds;
            return this;
        }

        /**
         * Adds the terminal state to enter after all sequence steps complete.
         *
         * @param state terminal state
         * @return this sequence
         */
        @Override
        public Sequence then(MechanismState state) {
            next = Objects.requireNonNull(state, "state");
            return this;
        }

        /**
         * Returns sequence steps.
         *
         * @return steps
         */
        public List<SequenceStep> steps() {
            return List.copyOf(steps);
        }

        /**
         * Returns terminal state.
         *
         * @return terminal state, or null
         */
        public MechanismState next() {
            return next;
        }

        /**
         * Returns sequence timeout.
         *
         * @return timeout duration, or positive infinity
         */
        public double timeoutSeconds() {
            return timeoutSeconds;
        }
    }

    public static final class When {
        private final StateCondition condition;

        private When(StateCondition condition) {
            this.condition = Objects.requireNonNull(condition, "condition");
        }

        public WhenBranch run(MechanismState state) {
            return new WhenBranch(condition, state);
        }

        public WhenBranch then(MechanismState state) {
            return run(state);
        }
    }

    public static final class Cycle implements MechanismState {
        private final List<CycleStep> steps;

        private Cycle() {
            this.steps = new ArrayList<>();
        }

        /**
         * Adds a step that advances after a fixed duration.
         *
         * @param seconds step duration
         * @param state step state
         * @return this cycle
         */
        public Cycle forTime(double seconds, MechanismState state) {
            steps.add(new CycleStep(state, ctx -> ctx.timeInStateSeconds() >= seconds));
            return this;
        }

        /**
         * Adds a step that advances when a condition is true.
         *
         * @param condition advancement condition
         * @param state step state
         * @return this cycle
         */
        public Cycle until(StateCondition condition, MechanismState state) {
            steps.add(new CycleStep(state, condition));
            return this;
        }

        /**
         * Adds a step that advances when a no-argument condition is true.
         *
         * @param condition advancement condition
         * @param state step state
         * @return this cycle
         */
        public Cycle until(BooleanSupplier condition, MechanismState state) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), state);
        }

        /**
         * Returns cycle steps.
         *
         * @return steps
         */
        public List<CycleStep> steps() {
            return List.copyOf(steps);
        }
    }

    public record CycleStep(MechanismState state, StateCondition advance) {
        public CycleStep {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(advance, "advance");
        }
    }

    public record SequenceStep(MechanismState state, StateCondition complete) {
        public SequenceStep {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(complete, "complete");
        }
    }

    private static List<MechanismState> copyStates(List<MechanismState> states) {
        Objects.requireNonNull(states, "states");
        List<MechanismState> copy = new ArrayList<>();
        for (MechanismState state : states) {
            copy.add(Objects.requireNonNull(state, "state"));
        }
        return List.copyOf(copy);
    }
}
