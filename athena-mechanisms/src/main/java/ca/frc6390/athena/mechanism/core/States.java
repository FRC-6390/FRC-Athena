package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
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
     * Creates a percent-output state for a specific control binding.
     *
     * @param control control binding
     * @param percent percent output
     * @return controlled percent state
     */
    public static MechanismState percent(ControlRef control, double percent) {
        return new ControlledPercent(control, percent);
    }

    /**
     * Creates a position state for a specific control binding.
     *
     * @param control control binding
     * @param position position target
     * @return controlled position state
     */
    public static MechanismState position(ControlRef control, double position) {
        return new ControlledPosition(control, position);
    }

    /**
     * Creates a velocity state for a specific control binding.
     *
     * @param control control binding
     * @param velocity velocity target
     * @return controlled velocity state
     */
    public static MechanismState velocity(ControlRef control, double velocity) {
        return new ControlledVelocity(control, velocity);
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
     * Starts a state sequence.
     *
     * @return sequence builder
     */
    public static Sequence sequence() {
        return new Sequence();
    }

    /**
     * Wraps a state in an until condition.
     *
     * @param state wrapped state
     * @param condition condition
     * @return conditional state
     */
    public static MechanismState until(MechanismState state, StateCondition condition) {
        return new Conditional(state, condition, null);
    }

    /**
     * Wraps a state in a no-argument until condition.
     *
     * @param state wrapped state
     * @param condition condition
     * @return conditional state
     */
    public static MechanismState until(MechanismState state, BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return until(state, ctx -> condition.getAsBoolean());
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

    public record ControlledPercent(ControlRef control, double percent) implements MechanismState, Output.Percent {
        public ControlledPercent {
            Objects.requireNonNull(control, "control");
        }
    }

    public record ControlledPosition(ControlRef control, double position)
            implements MechanismState, Output.Position {
        public ControlledPosition {
            Objects.requireNonNull(control, "control");
        }
    }

    public record ControlledVelocity(ControlRef control, double velocity)
            implements MechanismState, Output.Velocity {
        public ControlledVelocity {
            Objects.requireNonNull(control, "control");
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

        private Sequence() {
            this.steps = new ArrayList<>();
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
         * @param state step state
         * @param condition completion condition
         * @return this sequence
         */
        public Sequence until(MechanismState state, StateCondition condition) {
            steps.add(new SequenceStep(state, condition));
            return this;
        }

        /**
         * Adds a step that completes when a no-argument condition is true.
         *
         * @param state step state
         * @param condition completion condition
         * @return this sequence
         */
        public Sequence until(MechanismState state, BooleanSupplier condition) {
            Objects.requireNonNull(condition, "condition");
            return until(state, ctx -> condition.getAsBoolean());
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
    }

    public record SequenceStep(MechanismState state, StateCondition complete) {
        public SequenceStep {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(complete, "complete");
        }
    }
}
