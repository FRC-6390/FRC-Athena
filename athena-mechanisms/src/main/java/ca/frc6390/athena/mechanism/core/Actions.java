package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.mechanism.sysid.ControlSysId;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Factories for common mechanism Actions.
 */
public final class Actions {
    private static final double TIME_EPSILON_SECONDS = 1.0e-9;
    private static final Action NEUTRAL = new Neutral();

    private Actions() {
    }

    public static Action neutral() {
        return NEUTRAL;
    }

    static Action neutral(ControlBinding control) {
        return new ControlNeutral(control);
    }

    static Action neutral(MotorDevice motor) {
        return new MotorNeutral(motor);
    }

    static Action percent(MotorDevice motor, double percent) {
        return new MotorPercent(motor, percent);
    }

    static Action percent(MotorDevice motor, DoubleSupplier percent) {
        return new DynamicMotorPercent(motor, percent);
    }

    static Action percent(ControlBinding control, double percent) {
        return new ControlPercent(control, percent);
    }

    static Action percent(ControlBinding control, DoubleSupplier percent) {
        return new DynamicControlPercent(control, percent);
    }

    static Action voltage(MotorDevice motor, double volts) {
        return new MotorVoltage(motor, volts);
    }

    static Action voltage(MotorDevice motor, DoubleSupplier volts) {
        return new DynamicMotorVoltage(motor, volts);
    }

    static Action voltage(ControlBinding control, double volts) {
        return new ControlVoltage(control, volts);
    }

    static Action voltage(ControlBinding control, DoubleSupplier volts) {
        return new DynamicControlVoltage(control, volts);
    }

    static Action setPosition(EncoderDevice encoder, double position) {
        return new EncoderSetPosition(encoder, position);
    }

    static Action setYaw(ImuSource imu, double yawDegrees) {
        return new ImuSetYaw(imu, yawDegrees);
    }

    static Action setYaw(ImuSource imu, DoubleSupplier yawDegrees) {
        return new DynamicImuSetYaw(imu, yawDegrees);
    }

    static Action then(DeviceAction action, DeviceAction next) {
        return new Action.Then(mechanismAction(action), mechanismAction(next));
    }

    static Action timeout(DeviceAction action, double seconds) {
        return timeout(mechanismAction(action), seconds);
    }

    private static Action mechanismAction(DeviceAction action) {
        if (action instanceof Action mechanismAction) {
            return mechanismAction;
        }
        throw new IllegalArgumentException("Device action was not created by Athena mechanisms.");
    }

    static Action position(ControlBinding control, double position) {
        return new ControlPosition(control, position);
    }

    static Action position(ControlBinding control, DoubleSupplier position) {
        return new DynamicControlPosition(control, position);
    }

    static Action velocity(ControlBinding control, double velocity) {
        return new ControlVelocity(control, velocity);
    }

    static Action velocity(ControlBinding control, DoubleSupplier velocity) {
        return new DynamicControlVelocity(control, velocity);
    }

    public static Action fault(String reason) {
        return new Fault(reason);
    }

    public static Action dynamic(Supplier<Output> output) {
        Objects.requireNonNull(output, "output");
        return dynamic(ctx -> output.get());
    }

    /** Creates one direction of a control-binding SysId routine. */
    public static Action sysId(ControlSysId routine, SysIdState state) {
        return new ControlSysIdAction(routine, state);
    }

    /**
     * Publishes one scoped velocity contribution while returning the shared drivetrain action.
     * The channel is cleared automatically when this action stops running.
     */
    public static Action contributeVelocity(
            RobotVelocityPool.Channel channel,
            Supplier<RobotVelocity> velocity,
            Action driveAction) {
        return new VelocityContribution(channel, velocity, driveAction);
    }

    public static Action dynamic(Function<MechanismContext, Output> output) {
        return new DynamicOutput(output);
    }

    /**
     * Creates an action whose child tree is recomputed each cycle with required, stable ownership.
     * Ownership may contain devices, controls, mechanisms, actions, or iterables of those values.
     */
    public static Action compute(Supplier<Action> action, Object owner, Object... additionalOwnership) {
        Objects.requireNonNull(action, "action");
        return compute(context -> action.get(), owner, additionalOwnership);
    }

    /** Context-aware form of {@link #compute(Supplier, Object, Object...)}. */
    public static Action compute(
            Function<MechanismContext, Action> action,
            Object owner,
            Object... additionalOwnership) {
        Objects.requireNonNull(action, "action");
        List<Object> ownership = new ArrayList<>();
        ownership.add(Objects.requireNonNull(owner, "owner"));
        if (additionalOwnership != null) {
            for (Object value : additionalOwnership) {
                ownership.add(Objects.requireNonNull(value, "ownership"));
            }
        }
        return new Computed(ownership, action);
    }

    public static Action computeHardware(
            List<?> declarations,
            Function<ActionContext, Action> action) {
        return new HardwareComputed(declarations, action);
    }

    public static Action doOnce(Runnable action) {
        return new DoOnce(action);
    }


    public static Action doOnce(ActionBinding action) {
        return new RuntimeAction(action);
    }

    public static Action run(Runnable action) {
        return doOnce(action);
    }

    public static Action run(ActionBinding action) {
        return doOnce(action);
    }

    public static Action waitSeconds(double seconds) {
        return new WaitSeconds(seconds);
    }

    public static Action waitUntil(ActionCondition condition) {
        return new WaitUntil(condition);
    }

    public static Action waitUntil(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return waitUntil(ctx -> condition.getAsBoolean());
    }

    public static Action timeout(Action action, double seconds) {
        return new Timeout(action, seconds);
    }

    public static When when(ActionCondition condition) {
        return new When(condition);
    }

    public static When when(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return when(ctx -> condition.getAsBoolean());
    }

    public static Parallel parallel(Action... actions) {
        return new Parallel(List.of(actions));
    }

    public static Race race(Action... actions) {
        return new Race(List.of(actions));
    }

    public static Deadline deadline(Action primary, Action... others) {
        return new Deadline(primary, List.of(others));
    }

    public static Sequence sequence() {
        return new Sequence();
    }

    public static Cycle cycle() {
        return new Cycle();
    }

    public static Action until(ActionCondition condition, Action action) {
        return new Conditional(action, condition, null);
    }

    public static Action until(BooleanSupplier condition, Action action) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean(), action);
    }

    public static Action untilWithin(Action action, double tolerance) {
        return new WithinTolerance(action, tolerance);
    }

    public static Action then(Action action, Action next) {
        Objects.requireNonNull(next, "next");
        if (action instanceof Action.Conditional conditional) {
            return new Action.Conditional(conditional.action(), conditional.condition(), next);
        }
        if (action instanceof Conditional conditional) {
            return new Conditional(conditional.action(), conditional.condition(), next);
        }
        if (action instanceof Sequence sequence) {
            return sequence.then(next);
        }
        return new Then(action, next);
    }

    public record Neutral() implements Action, Output.Neutral {
    }

    public record ControlNeutral(ControlBinding control) implements Action, Output.Neutral {
        public ControlNeutral {
            Objects.requireNonNull(control, "control");
        }
    }

    public record MotorNeutral(MotorDevice motor) implements Action, Output.Neutral {
        public MotorNeutral {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record MotorPercent(MotorDevice motor, double percent) implements Action, Output.Percent {
        public MotorPercent {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record DynamicMotorPercent(MotorDevice motor, DoubleSupplier percentSupplier) implements Action, Output.Percent {
        public DynamicMotorPercent {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
        }

        @Override
        public double percent() {
            return percentSupplier.getAsDouble();
        }
    }

    public record MotorVoltage(MotorDevice motor, double volts) implements Action, Output.Voltage {
        public MotorVoltage {
            Objects.requireNonNull(motor, "motor");
        }
    }

    public record DynamicMotorVoltage(MotorDevice motor, DoubleSupplier voltsSupplier) implements Action, Output.Voltage {
        public DynamicMotorVoltage {
            Objects.requireNonNull(motor, "motor");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
        }

        @Override
        public double volts() {
            return voltsSupplier.getAsDouble();
        }
    }

    public record ControlPercent(ControlBinding control, double percent) implements Action, Output.Percent {
        public ControlPercent {
            Objects.requireNonNull(control, "control");
            requireControlOutput(control);
            requireFinite(percent, "Control percent");
        }
    }

    public record DynamicControlPercent(ControlBinding control, DoubleSupplier percentSupplier) implements Action, Output.Percent {
        public DynamicControlPercent {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
            requireControlOutput(control);
        }

        @Override
        public double percent() {
            return requireFinite(percentSupplier.getAsDouble(), "Control percent");
        }
    }

    public record ControlVoltage(ControlBinding control, double volts) implements Action, Output.Voltage {
        public ControlVoltage {
            Objects.requireNonNull(control, "control");
            requireControlOutput(control);
            requireFinite(volts, "Control voltage");
        }
    }

    public record DynamicControlVoltage(ControlBinding control, DoubleSupplier voltsSupplier) implements Action, Output.Voltage {
        public DynamicControlVoltage {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
            requireControlOutput(control);
        }

        @Override
        public double volts() {
            return requireFinite(voltsSupplier.getAsDouble(), "Control voltage");
        }
    }

    public record ControlSysIdAction(ControlSysId routine, SysIdState state) implements Action {
        public ControlSysIdAction {
            Objects.requireNonNull(routine, "routine");
            Objects.requireNonNull(state, "state");
            if (state == SysIdState.NONE) {
                throw new IllegalArgumentException("A SysId action requires an active test state.");
            }
        }

        ControlSysIdVoltage output(double elapsedSeconds) {
            return new ControlSysIdVoltage(routine, state, routine.voltage(state, elapsedSeconds));
        }
    }

    public record ControlSysIdVoltage(
            ControlSysId routine,
            SysIdState state,
            double volts) implements Action, Output.Voltage {
        public ControlSysIdVoltage {
            Objects.requireNonNull(routine, "routine");
            Objects.requireNonNull(state, "state");
            requireFinite(volts, "SysId voltage");
        }
    }

    public record ControlPosition(ControlBinding control, double position) implements Action, Output.Position {
        public ControlPosition {
            Objects.requireNonNull(control, "control");
            requireControlMode(control, ControlMode.POSITION);
            requireFinite(position, "Control position");
        }
    }

    public record DynamicControlPosition(ControlBinding control, DoubleSupplier positionSupplier) implements Action, Output.Position {
        public DynamicControlPosition {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(positionSupplier, "positionSupplier");
            requireControlMode(control, ControlMode.POSITION);
        }

        @Override
        public double position() {
            return requireFinite(positionSupplier.getAsDouble(), "Control position");
        }
    }

    public record ControlVelocity(ControlBinding control, double velocity) implements Action, Output.Velocity {
        public ControlVelocity {
            Objects.requireNonNull(control, "control");
            requireControlMode(control, ControlMode.VELOCITY);
            requireFinite(velocity, "Control velocity");
        }
    }

    public record DynamicControlVelocity(ControlBinding control, DoubleSupplier velocitySupplier) implements Action, Output.Velocity {
        public DynamicControlVelocity {
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(velocitySupplier, "velocitySupplier");
            requireControlMode(control, ControlMode.VELOCITY);
        }

        @Override
        public double velocity() {
            return requireFinite(velocitySupplier.getAsDouble(), "Control velocity");
        }
    }

    public record Fault(String reason) implements Action, Output.Fault {
        public Fault {
            reason = reason == null ? "" : reason;
        }
    }

    public record EncoderSetPosition(EncoderDevice encoder, double position) implements Action {
        public EncoderSetPosition {
            Objects.requireNonNull(encoder, "encoder");
            if (!Double.isFinite(position)) {
                throw new IllegalArgumentException("Encoder position must be finite.");
            }
        }

        @Override
        public void apply(ActionContext context) {
            context.encoder(encoder).setPositionRotations(encoder.rotationsFromPosition(position));
        }
    }

    public interface ImuYawMutation extends Action {
        ImuSource imu();

        double requestedYawDegrees();

        @Override
        default void apply(ActionContext context) {
            imu().applyYaw(context, requestedYawDegrees());
        }
    }

    public record VelocityContribution(
            RobotVelocityPool.Channel channel,
            Supplier<RobotVelocity> velocity,
            Action driveAction) implements Action {
        public VelocityContribution {
            Objects.requireNonNull(channel, "channel");
            Objects.requireNonNull(velocity, "velocity");
            Objects.requireNonNull(driveAction, "driveAction");
        }

        RobotVelocity sample() {
            return Objects.requireNonNull(velocity.get(), "velocity supplier returned null");
        }
    }

    public record ImuSetYaw(ImuSource imu, double yawDegrees) implements ImuYawMutation {
        public ImuSetYaw {
            Objects.requireNonNull(imu, "imu");
            if (!Double.isFinite(yawDegrees)) {
                throw new IllegalArgumentException("IMU yaw must be finite.");
            }
        }
        @Override
        public double requestedYawDegrees() {
            return yawDegrees;
        }
    }

    public record DynamicImuSetYaw(ImuSource imu, DoubleSupplier yawDegrees) implements ImuYawMutation {
        public DynamicImuSetYaw {
            Objects.requireNonNull(imu, "imu");
            Objects.requireNonNull(yawDegrees, "yawDegrees");
        }

        @Override
        public double requestedYawDegrees() {
            return requireFinite(yawDegrees.getAsDouble(), "IMU yaw");
        }
    }

    public record DoOnce(Runnable action) implements Action {
        public DoOnce {
            Objects.requireNonNull(action, "action");
        }
    }

    public record RuntimeAction(ActionBinding action) implements Action {
        public RuntimeAction {
            Objects.requireNonNull(action, "action");
        }
    }

    public record WaitSeconds(double seconds) implements Action {
        public boolean complete(MechanismContext ctx) {
            return hasElapsed(ctx.timeInStateSeconds(), seconds);
        }
    }

    public record WaitUntil(ActionCondition condition) implements Action {
        public WaitUntil {
            Objects.requireNonNull(condition, "condition");
        }

        public boolean complete(MechanismContext ctx) {
            return condition.test(ctx);
        }
    }

    public record Timeout(Action action, double seconds) implements Action {
        public Timeout {
            Objects.requireNonNull(action, "action");
        }

        public boolean expired(MechanismContext ctx) {
            return hasElapsed(ctx.timeInStateSeconds(), seconds);
        }
    }

    public record Choice(ActionCondition condition, Action active, Action inactive) implements Action {
        public Choice {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
            Objects.requireNonNull(inactive, "inactive");
        }

        public Action choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : inactive;
        }
    }

    public record WhenBranch(ActionCondition condition, Action active) implements Action {
        public WhenBranch {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
        }

        public Choice otherwise(Action inactive) {
            return new Choice(condition, active, inactive);
        }

        public Action choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : neutral();
        }
    }

    public record Parallel(List<Action> Actions) implements Action {
        public Parallel {
            Actions = copyStates(Actions);
        }
    }

    public record Race(List<Action> Actions) implements Action {
        public Race {
            Actions = copyStates(Actions);
        }
    }

    public static final class Deadline implements Action {
        private final Action primary;
        private final List<Action> others;
        private final List<Action> actions;

        public Deadline(Action primary, List<Action> others) {
            this.primary = Objects.requireNonNull(primary, "primary");
            this.others = copyStates(others);
            this.actions = new DeadlineActions(this.primary, this.others);
        }

        public Action primary() {
            return primary;
        }

        public List<Action> others() {
            return others;
        }

        public List<Action> Actions() {
            return actions;
        }
    }

    public record DynamicOutput(Function<MechanismContext, Output> output) implements Action {
        public DynamicOutput {
            Objects.requireNonNull(output, "output");
        }
    }

    public record Computed(List<?> ownership, Function<MechanismContext, Action> action) implements Action {
        public Computed {
            ownership = List.copyOf(Objects.requireNonNull(ownership, "ownership"));
            if (ownership.isEmpty()) throw new IllegalArgumentException("Computed actions require ownership.");
            Objects.requireNonNull(action, "action");
        }

        public Action evaluate(MechanismContext context) {
            return Objects.requireNonNull(action.apply(context), "Computed action returned null.");
        }
    }

    public record HardwareComputed(List<?> declarations, Function<ActionContext, Action> action) implements Action {
        public HardwareComputed {
            declarations = declarations == null ? List.of() : List.copyOf(declarations);
            Objects.requireNonNull(action, "action");
        }

        public Action evaluate(ActionContext context) {
            return Objects.requireNonNull(action.apply(context), "Hardware-computed action returned null.");
        }
    }

    public record Conditional(Action action, ActionCondition condition, Action next) implements Action {
        public Conditional {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(condition, "condition");
        }
    }

    public record WithinTolerance(Action action, double tolerance) implements Action {
        public WithinTolerance {
            Objects.requireNonNull(action, "action");
            if (!supportsTolerance(action)) {
                throw new IllegalArgumentException(
                        "untilWithin requires one position or velocity control action.");
            }
            if (!Double.isFinite(tolerance) || tolerance < 0.0) {
                throw new IllegalArgumentException("Control tolerance must be finite and non-negative.");
            }
        }
    }

    public record Then(Action action, Action next) implements Action {
        public Then {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(next, "next");
        }
    }

    public static final class Sequence implements Action {
        private final List<SequenceStep> steps = new ArrayList<>();
        private final List<SequenceStep> stepView = Collections.unmodifiableList(steps);
        private final List<Action> started = new ArrayList<>();
        private Action next;
        private double timeoutSeconds = Double.POSITIVE_INFINITY;

        private Sequence() {
        }

        public Sequence run(Action action) {
            steps.add(new SequenceStep(withStartedUntilComplete(action), ctx -> false, false));
            return this;
        }

        public Sequence forTime(double seconds, Action action) {
            steps.add(new SequenceStep(
                    alongsideStarted(action),
                    ctx -> hasElapsed(ctx.timeInStateSeconds(), seconds),
                    true));
            return this;
        }

        public Sequence until(ActionCondition condition, Action action) {
            steps.add(new SequenceStep(alongsideStarted(action), condition, true));
            return this;
        }

        public Sequence until(BooleanSupplier condition, Action action) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), action);
        }

        public Sequence doOnce(Runnable action) {
            steps.add(new SequenceStep(withStartedUntilComplete(Actions.doOnce(action)), ctx -> true, true));
            return this;
        }

        public Sequence doOnce(ActionBinding action) {
            steps.add(new SequenceStep(withStartedUntilComplete(Actions.doOnce(action)), ctx -> true, true));
            return this;
        }

        public Sequence doOnce(DeviceAction action) {
            return doOnce((ActionBinding) action);
        }

        public Sequence waitSeconds(double seconds) {
            steps.add(new SequenceStep(
                    alongsideStarted(Actions.waitSeconds(seconds)),
                    ctx -> hasElapsed(ctx.timeInStateSeconds(), seconds),
                    true));
            return this;
        }

        public Sequence waitUntil(ActionCondition condition) {
            steps.add(new SequenceStep(alongsideStarted(Actions.waitUntil(condition)), condition, true));
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

        /**
         * Starts an action that remains active alongside every following sequence stage.
         * Started actions are released with the containing sequence.
         */
        public Sequence start(Action action) {
            Action safeAction = Objects.requireNonNull(action, "action");
            for (Action existing : started) {
                if (existing == safeAction) return this;
            }
            started.add(safeAction);
            return this;
        }

        /** Stops carrying a previously started action into subsequent stages. */
        public Sequence stop(Action action) {
            Action safeAction = Objects.requireNonNull(action, "action");
            started.removeIf(existing -> existing == safeAction);
            return this;
        }

        @Override
        public Sequence then(Action action) {
            Action safeAction = Objects.requireNonNull(action, "action");
            safeAction = withStartedUntilComplete(safeAction);
            next = next == null ? safeAction : Actions.then(next, safeAction);
            return this;
        }

        public List<SequenceStep> steps() {
            return stepView;
        }

        public Action next() {
            return next;
        }

        public double timeoutSeconds() {
            return timeoutSeconds;
        }

        private Action alongsideStarted(Action action) {
            Action safeAction = Objects.requireNonNull(action, "action");
            if (started.isEmpty()) return safeAction;
            Action[] actions = new Action[started.size() + 1];
            actions[0] = safeAction;
            for (int index = 0; index < started.size(); index++) {
                actions[index + 1] = started.get(index);
            }
            return Actions.parallel(actions);
        }

        private Action withStartedUntilComplete(Action action) {
            Action safeAction = Objects.requireNonNull(action, "action");
            if (started.isEmpty()) return safeAction;
            return Actions.deadline(safeAction, started.toArray(Action[]::new));
        }
    }

    public static final class When {
        private final ActionCondition condition;

        private When(ActionCondition condition) {
            this.condition = Objects.requireNonNull(condition, "condition");
        }

        public WhenBranch run(Action action) {
            return new WhenBranch(condition, action);
        }

        public WhenBranch then(Action action) {
            return run(action);
        }
    }

    public static final class Cycle implements Action {
        private final List<CycleStep> steps = new ArrayList<>();
        private final List<CycleStep> stepView = Collections.unmodifiableList(steps);

        private Cycle() {
        }

        public Cycle run(Action action) {
            steps.add(new CycleStep(action, ctx -> false));
            return this;
        }

        public Cycle forTime(double seconds, Action action) {
            steps.add(new CycleStep(action, ctx -> hasElapsed(ctx.timeInStateSeconds(), seconds)));
            return this;
        }

        public Cycle until(ActionCondition condition, Action action) {
            steps.add(new CycleStep(action, condition));
            return this;
        }

        public Cycle until(BooleanSupplier condition, Action action) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), action);
        }

        public List<CycleStep> steps() {
            return stepView;
        }
    }

    public record CycleStep(Action action, ActionCondition advance) {
        public CycleStep {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(advance, "advance");
        }
    }

    public record SequenceStep(
            Action action,
            ActionCondition complete,
            boolean advancesIndependently) {
        public SequenceStep {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(complete, "complete");
        }
    }

    static void validate(Action action) {
        validate(action, Collections.newSetFromMap(new IdentityHashMap<>()));
    }

    static boolean hasElapsed(double elapsedSeconds, double durationSeconds) {
        return elapsedSeconds + TIME_EPSILON_SECONDS >= durationSeconds;
    }

    private static void validate(Action action, Set<Action> visited) {
        if (action == null || !visited.add(action)) return;
        if (action instanceof Sequence sequence) {
            validateSequence(sequence);
            for (SequenceStep step : sequence.steps()) validate(step.action(), visited);
            validate(sequence.next(), visited);
        } else if (action instanceof Parallel parallel) {
            parallel.Actions().forEach(child -> validate(child, visited));
        } else if (action instanceof Race race) {
            race.Actions().forEach(child -> validate(child, visited));
        } else if (action instanceof Deadline deadline) {
            deadline.Actions().forEach(child -> validate(child, visited));
        } else if (action instanceof Cycle cycle) {
            cycle.steps().forEach(step -> validate(step.action(), visited));
        } else if (action instanceof Timeout timeout) {
            validate(timeout.action(), visited);
        } else if (action instanceof WithinTolerance within) {
            validate(within.action(), visited);
        } else if (action instanceof Conditional conditional) {
            validate(conditional.action(), visited);
            validate(conditional.next(), visited);
        } else if (action instanceof Action.Conditional conditional) {
            validate(conditional.action(), visited);
            validate(conditional.next(), visited);
        } else if (action instanceof Then then) {
            validateThenSource(then.action());
            validate(then.action(), visited);
            validate(then.next(), visited);
        } else if (action instanceof Action.Then then) {
            validateThenSource(then.action());
            validate(then.action(), visited);
            validate(then.next(), visited);
        } else if (action instanceof Choice choice) {
            validate(choice.active(), visited);
            validate(choice.inactive(), visited);
        } else if (action instanceof WhenBranch branch) {
            validate(branch.active(), visited);
        }
    }

    private static void validateSequence(Sequence sequence) {
        if (Double.isFinite(sequence.timeoutSeconds())) return;
        List<SequenceStep> steps = sequence.steps();
        for (int index = 0; index < steps.size(); index++) {
            SequenceStep step = steps.get(index);
            boolean hasFollowingStage = index + 1 < steps.size() || sequence.next() != null;
            if (hasFollowingStage
                    && !step.advancesIndependently()
                    && completion(step.action(), Collections.newSetFromMap(new IdentityHashMap<>()))
                            == Completion.NEVER) {
                throw new IllegalArgumentException(
                        "Sequence cannot advance past run(...) because that action is continuous. "
                                + "Use forTime(...), until(...), or start(...) for an action that should remain active.");
            }
        }
    }

    private static void validateThenSource(Action action) {
        if (completion(action, Collections.newSetFromMap(new IdentityHashMap<>())) == Completion.NEVER) {
            throw new IllegalArgumentException(
                    "Action chain cannot advance past then(...) because the preceding action is continuous. "
                            + "Complete it with forTime(...), timeout(...), until(...), or untilWithin(...).");
        }
    }

    private static Completion completion(Action action, Set<Action> visited) {
        if (action == null || !visited.add(action)) return Completion.UNKNOWN;
        if (action instanceof Output) return Completion.NEVER;
        if (action instanceof DoOnce || action instanceof RuntimeAction
                || action instanceof EncoderSetPosition || action instanceof ImuYawMutation
                || action instanceof WaitSeconds || action instanceof WaitUntil
                || action instanceof Timeout || action instanceof WithinTolerance
                || action instanceof Conditional || action instanceof Action.Conditional) {
            return Completion.POSSIBLE;
        }
        if (action instanceof Sequence sequence) {
            if (Double.isFinite(sequence.timeoutSeconds())) return Completion.POSSIBLE;
            Completion result = Completion.POSSIBLE;
            for (SequenceStep step : sequence.steps()) {
                if (step.advancesIndependently()) continue;
                Completion child = completion(step.action(), visited);
                if (child == Completion.NEVER) return Completion.NEVER;
                if (child == Completion.UNKNOWN) result = Completion.UNKNOWN;
            }
            if (sequence.next() != null) {
                Completion next = completion(sequence.next(), visited);
                if (next == Completion.NEVER) return Completion.NEVER;
                if (next == Completion.UNKNOWN) result = Completion.UNKNOWN;
            }
            return result;
        }
        if (action instanceof Parallel parallel) {
            if (parallel.Actions().isEmpty()) return Completion.NEVER;
            Completion result = Completion.POSSIBLE;
            for (Action child : parallel.Actions()) {
                Completion completion = completion(child, visited);
                if (completion == Completion.NEVER) return Completion.NEVER;
                if (completion == Completion.UNKNOWN) result = Completion.UNKNOWN;
            }
            return result;
        }
        if (action instanceof Deadline deadline) {
            return completion(deadline.primary(), visited);
        }
        if (action instanceof Race race) {
            boolean unknown = false;
            for (Action child : race.Actions()) {
                Completion completion = completion(child, visited);
                if (completion == Completion.POSSIBLE) return Completion.POSSIBLE;
                unknown |= completion == Completion.UNKNOWN;
            }
            return unknown ? Completion.UNKNOWN : Completion.NEVER;
        }
        if (action instanceof Then then) {
            return both(completion(then.action(), visited), completion(then.next(), visited));
        }
        if (action instanceof Action.Then then) {
            return both(completion(then.action(), visited), completion(then.next(), visited));
        }
        if (action instanceof VelocityContribution contribution) {
            return completion(contribution.driveAction(), visited);
        }
        return Completion.UNKNOWN;
    }

    private static Completion both(Completion first, Completion second) {
        if (first == Completion.NEVER || second == Completion.NEVER) return Completion.NEVER;
        if (first == Completion.UNKNOWN || second == Completion.UNKNOWN) return Completion.UNKNOWN;
        return Completion.POSSIBLE;
    }

    private enum Completion {
        POSSIBLE,
        NEVER,
        UNKNOWN
    }

    private static List<Action> copyStates(List<Action> Actions) {
        Objects.requireNonNull(Actions, "Actions");
        List<Action> copy = new ArrayList<>();
        for (Action action : Actions) {
            copy.add(Objects.requireNonNull(action, "action"));
        }
        return List.copyOf(copy);
    }

    private static boolean supportsTolerance(Action action) {
        if (action instanceof ControlPosition
                || action instanceof DynamicControlPosition
                || action instanceof ControlVelocity
                || action instanceof DynamicControlVelocity
                || action instanceof InterpolatedControlAction) {
            return true;
        }
        return false;
    }

    private static void requireControlOutput(ControlBinding control) {
        if (control.output() == null && control.sink() == null) {
            throw new IllegalStateException("Control actions require an output destination.");
        }
    }

    private static void requireControlMode(ControlBinding control, ControlMode required) {
        requireControlOutput(control);
        if (control.mode() != required) {
            throw new IllegalStateException(required.name().toLowerCase(java.util.Locale.ROOT)
                    + " action requires a matching control binding.");
        }
    }

    private static double requireFinite(double value, String description) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(description + " must be finite.");
        }
        return value;
    }

    private static final class DeadlineActions extends java.util.AbstractList<Action> {
        private final Action primary;
        private final List<Action> others;

        private DeadlineActions(Action primary, List<Action> others) {
            this.primary = primary;
            this.others = others;
        }

        @Override
        public Action get(int index) {
            if (index == 0) {
                return primary;
            }
            return others.get(index - 1);
        }

        @Override
        public int size() {
            return others.size() + 1;
        }
    }
}
