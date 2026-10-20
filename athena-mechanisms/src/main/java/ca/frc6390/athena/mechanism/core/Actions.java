package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.effort.EffortCarrier;
import ca.frc6390.athena.mechanism.effort.EffortRequest;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;
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

    /** Builds one labelled child, or null when the child is absent. */
    static Action.Child child(String role, Action action) {
        return action == null ? null : new Action.Child(role, action);
    }

    /** Collects the non-null children in order. */
    static List<Action.Child> childrenOf(Action.Child... children) {
        List<Action.Child> out = new ArrayList<>(children.length);
        for (Action.Child child : children) {
            if (child != null) out.add(child);
        }
        return List.copyOf(out);
    }

    /** Adds every action of a list under one role. */
    static void addChildren(List<Action.Child> out, String role, List<? extends Action> actions) {
        for (Action action : actions) {
            out.add(new Action.Child(role, action));
        }
    }

    /** Label suffix describing a control's effort limit, empty when it has none. */
    static String effortLabel(EffortRequest effort) {
        return effort == null || !effort.limited() ? "" : " effort " + effort.currentLimitAmps() + "A";
    }

    public static Action neutral() {
        return NEUTRAL;
    }

    static Action neutral(ControlBinding<?> control) {
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

    static Action percent(ControlBinding<?> control, double percent) {
        return new ControlPercent(control, percent);
    }

    static Action percent(ControlBinding<?> control, DoubleSupplier percent) {
        return new DynamicControlPercent(control, percent);
    }

    static Action voltage(MotorDevice motor, double volts) {
        return new MotorVoltage(motor, volts);
    }

    static Action voltage(MotorDevice motor, DoubleSupplier volts) {
        return new DynamicMotorVoltage(motor, volts);
    }

    static Action voltage(ControlBinding<?> control, double volts) {
        return new ControlVoltage(control, volts);
    }

    static Action voltage(ControlBinding<?> control, DoubleSupplier volts) {
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
        return new Then(mechanismAction(action), mechanismAction(next));
    }

    static Action timeout(DeviceAction action, Time duration) {
        return timeout(mechanismAction(action), duration);
    }

    private static Action mechanismAction(DeviceAction action) {
        if (action instanceof Action mechanismAction) {
            return mechanismAction;
        }
        throw new IllegalArgumentException("Device action was not created by Athena mechanisms.");
    }

    static Action position(ControlBinding<?> control, double position) {
        return new ControlPosition(control, position);
    }

    static Action position(ControlBinding<?> control, DoubleSupplier position) {
        return new DynamicControlPosition(control, position);
    }

    static Action velocity(ControlBinding<?> control, double velocity) {
        return new ControlVelocity(control, velocity);
    }

    static Action velocity(ControlBinding<?> control, DoubleSupplier velocity) {
        return new DynamicControlVelocity(control, velocity);
    }

    /**
     * Reports a deliberate fault through the runtime failure reporter and completes immediately. Use it
     * as a terminal step of an {@link #attempt(Action) attempt} chain, or to flag an unrecoverable state.
     */
    public static Action fault(String reason) {
        String message = reason == null || reason.isBlank() ? "Action fault" : reason;
        return new DoOnce(() -> FaultIsolation.report(message));
    }

    public static Action dynamic(Supplier<Output> output) {
        Objects.requireNonNull(output, "output");
        return dynamic(ctx -> output.get());
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

    /**
     * Hardware declaration actions ({@code encoder.setPosition(0.0)}, {@code motor.neutral()}) are
     * mechanism Actions. They are scheduled as that Action inside the enclosing composition, not run as
     * a plain Runnable. This overload also resolves a declaration argument that would otherwise be
     * ambiguous between {@link #doOnce(Runnable)} and {@link #doOnce(ActionBinding)}.
     */
    public static Action doOnce(DeviceAction action) {
        return mechanismAction(action);
    }

    public static Action run(Runnable action) {
        return doOnce(action);
    }

    /**
     * Starts a lifecycle Action builder for external imperative behaviour: {@code onStart}, a per-cycle
     * {@code execute}, an {@code until} finish condition, and an {@code onEnd(interrupted)} callback. See
     * {@link LifecycleAction}.
     *
     * @param name diagnostic name
     * @return lifecycle Action builder
     */
    public static LifecycleAction.Builder lifecycle(String name) {
        return LifecycleAction.builder(name);
    }

    public static Action run(ActionBinding action) {
        return doOnce(action);
    }

    /** Declaration overload of {@link #run(ActionBinding)}; see {@link #doOnce(DeviceAction)}. */
    public static Action run(DeviceAction action) {
        return mechanismAction(action);
    }

    public static Action waitSeconds(Time duration) {
        return new WaitSeconds(duration);
    }

    /**
     * Completes once the stall detector has seen a commanded, non-moving motor for its configured
     * duration. Debounce timing restarts each time this action starts, so the same detector can be
     * reused by several actions. Combine it with the action that drives the motor, for example
     * {@code Actions.untilStalled(signal, arm.voltage(-2.0))}.
     */
    public static Action waitUntilStalled(MotorStallSignal signal) {
        return new WaitUntilStalled(signal);
    }

    /**
     * Runs {@code action} until {@code signal} reports a stall, then completes. Typical homing:
     * {@code Actions.untilStalled(motor.stall(), arm.voltage(-2.0)).then(motor.encoder().setPosition(0.0))}.
     */
    public static Action untilStalled(MotorStallSignal signal, Action action) {
        return new Race(List.of(Objects.requireNonNull(action, "action"), waitUntilStalled(signal)));
    }

    static Action untilStalled(DeviceAction action, MotorStallSignal signal) {
        return untilStalled(signal, mechanismAction(action));
    }

    /**
     * Picks {@code active} or {@code inactive} once, when this action starts, and runs that branch
     * to completion. Unlike {@link #when(ActionCondition)}, the condition is not re-checked while
     * the chosen branch runs.
     */
    public static Action either(ActionCondition condition, Action active, Action inactive) {
        return new Either(condition, active, Objects.requireNonNull(inactive, "inactive"));
    }

    public static Action either(BooleanSupplier condition, Action active, Action inactive) {
        Objects.requireNonNull(condition, "condition");
        return either(ctx -> condition.getAsBoolean(), active, inactive);
    }

    /** Runs {@code action} when the condition is true at start; otherwise completes immediately. */
    public static Action onlyIf(ActionCondition condition, Action action) {
        return new Either(condition, action, null);
    }

    public static Action onlyIf(BooleanSupplier condition, Action action) {
        Objects.requireNonNull(condition, "condition");
        return onlyIf(ctx -> condition.getAsBoolean(), action);
    }

    public static Action waitUntil(ActionCondition condition) {
        return new WaitUntil(condition);
    }

    public static Action waitUntil(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return waitUntil(ctx -> condition.getAsBoolean());
    }

    public static Action timeout(Action action, Time duration) {
        return new Timeout(action, duration);
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

    /** Typed tolerance; converted with the action's control binding unit at declaration. */
    public static Action untilWithin(Action action, Measure<?> tolerance) {
        Objects.requireNonNull(tolerance, "tolerance");
        ControlBinding<?> control = toleranceControl(action);
        return new WithinTolerance(action, control.modeValue(tolerance));
    }

    /**
     * Requires the measurement to stay inside the tolerance window for {@code seconds} continuously.
     * The timer restarts whenever the measurement leaves the window.
     */
    public static Action settled(Action action, Time duration) {
        WithinTolerance within = requireWithin(action, "settled");
        double seconds = requireDuration(duration.in(Seconds), "Settle duration");
        return new WithinTolerance(within.action(), within.tolerance(), seconds, within.maxVelocity());
    }

    /** Typed velocity limit: a rate of the control binding's unit per second. */
    public static Action maxVelocity(Action action, Measure<?> maxVelocity) {
        Objects.requireNonNull(maxVelocity, "maxVelocity");
        WithinTolerance within = requireWithin(action, "maxVelocity");
        double limit = toleranceControl(within.action()).rateValue(maxVelocity);
        return new WithinTolerance(within.action(), within.tolerance(), within.settleSeconds(), limit);
    }

    private static WithinTolerance requireWithin(Action action, String method) {
        Objects.requireNonNull(action, "action");
        if (action instanceof WithinTolerance within) {
            return within;
        }
        throw new IllegalArgumentException(method + " applies to untilWithin(...); call untilWithin first.");
    }

    private static boolean isVelocityControl(Action action) {
        if (action instanceof ControlVelocity || action instanceof DynamicControlVelocity) {
            return true;
        }
        return action instanceof InterpolatedControlAction interpolated
                && interpolated.control().mode() == ControlMode.VELOCITY;
    }

    private static ControlBinding<?> toleranceControl(Action action) {
        if (action instanceof ControlPosition position) {
            return position.control();
        }
        if (action instanceof DynamicControlPosition position) {
            return position.control();
        }
        if (action instanceof ControlVelocity velocity) {
            return velocity.control();
        }
        if (action instanceof DynamicControlVelocity velocity) {
            return velocity.control();
        }
        if (action instanceof InterpolatedControlAction interpolated) {
            return interpolated.control();
        }
        throw new IllegalArgumentException("untilWithin requires one position or velocity control action.");
    }

    public static Action then(Action action, Action next) {
        Objects.requireNonNull(next, "next");
        if (action instanceof Conditional conditional) {
            return conditional.then(next);
        }
        if (action instanceof Sequence sequence) {
            return sequence.then(next);
        }
        return new Then(action, next);
    }

    /**
     * Root-level interrupt policy. Request it instead of the inner Action; the scheduler holds newer
     * requests for overlapping resources while the inner Action does not allow interruption.
     * Nested use is rejected when the Action tree is compiled.
     */
    public record Interruptible(Action action, Interrupt interrupt) implements Action {

        @Override
        public List<Child> children() {
            return childrenOf(child("body", action));
        }

        @Override
        public String describe() {
            return "INTERRUPT " + interrupt;
        }

        public Interruptible {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(interrupt, "interrupt");
            if (action instanceof Interruptible || action instanceof Abort) {
                throw new IllegalArgumentException("Interrupt policies and abort are root-level requests and cannot be nested.");
            }
            if (interrupt.restrictive() && !isBounded(action)) {
                throw new IllegalArgumentException("Interrupt policy " + interrupt
                        + " requires a bounded Action: put every continuous step inside timeout(...)/forTime(...),"
                        + " or give the Action .atMost(Duration).");
            }
        }
    }

    /** Request-only Action that cancels every committed lease touching a mechanism, ignoring policies. */
    public record Abort(Mechanism mechanism) implements Action {
        public Abort {
            Objects.requireNonNull(mechanism, "mechanism");
        }
    }

    /**
     * Returns an Action that cancels every committed Action on {@code mechanism}, overriding interrupt
     * policies. Request it like any other Action; it does not occupy a lease itself.
     */
    public static Action abort(Mechanism mechanism) {
        return new Abort(mechanism);
    }

    /** Wraps an Action with a root interrupt policy, validating bounded commitment at declaration. */
    static Action interruptible(Action action, Interrupt interrupt) {
        return new Interruptible(action, interrupt);
    }

    /**
     * Static analysis for the bounded commitment rule. An Action is bounded when it is guaranteed to
     * finish on its own: every continuous leaf (position, velocity, percent, voltage, wait-until,
     * computed output, unknown Action) must sit inside a timeout, a timed sequence step, or be
     * bounded by {@code .atMost(Duration)}. Instantaneous Actions (neutral, fault, run, waitSeconds)
     * are bounded.
     */
    public static boolean isBounded(Action action) {
        return bounded(Objects.requireNonNull(action, "action"));
    }

    private static boolean bounded(Action action) {
        if (action == null) return true;
        if (action instanceof Timeout || action instanceof WaitSeconds || action instanceof DoOnce
                || action instanceof RuntimeAction || action instanceof EncoderSetPosition
                || action instanceof ImuYawMutation || action instanceof Neutral
                || action instanceof ControlNeutral || action instanceof MotorNeutral
                || action instanceof Fault || action instanceof Abort) {
            return true;
        }
        if (action instanceof LifecycleAction lifecycle) return lifecycle.finishes();
        if (action instanceof Interruptible interruptible) return bounded(interruptible.action());
        if (action instanceof WithinTolerance within) return bounded(within.action());
        if (action instanceof Conditional conditional) {
            return bounded(conditional.action()) && bounded(conditional.next());
        }
        if (action instanceof Then then) return bounded(then.action()) && bounded(then.next());
        if (action instanceof Sequence sequence) {
            if (Double.isFinite(sequence.timeoutSeconds())) return true;
            for (SequenceStep step : sequence.steps()) {
                if (!sequence.boundedStep(step) && !bounded(step.action())) return false;
            }
            return bounded(sequence.next());
        }
        if (action instanceof Parallel parallel) {
            return parallel.Actions().stream().allMatch(Actions::bounded);
        }
        if (action instanceof Race race) {
            return race.Actions().isEmpty() || race.Actions().stream().anyMatch(Actions::bounded);
        }
        if (action instanceof Deadline deadline) return bounded(deadline.primary());
        if (action instanceof Choice choice) return bounded(choice.active()) && bounded(choice.inactive());
        if (action instanceof Either either) return bounded(either.active()) && bounded(either.inactive());
        return false;
    }

    private static void rejectNestedRootOnly(Action action) {
        if (action instanceof Interruptible || action instanceof Abort) {
            throw new IllegalArgumentException("Interrupt policies and abort are root-level requests; request them directly instead of nesting them in another Action.");
        }
    }

    /** Returns {@code action} with the policy override merged in (non-null components win). */
    static Action withMotionPolicy(Action action, MotionPolicy update) {
        Objects.requireNonNull(action, "action");
        if (action instanceof ControlPosition position) {
            return new ControlPosition(position.control(), position.position(),
                    merge(position.policy(), update), position.effort());
        }
        if (action instanceof DynamicControlPosition position) {
            return new DynamicControlPosition(position.control(), position.positionSupplier(),
                    merge(position.policy(), update), position.effort());
        }
        if (action instanceof ControlVelocity velocity) {
            return new ControlVelocity(velocity.control(), velocity.velocity(),
                    merge(velocity.policy(), update), velocity.effort());
        }
        if (action instanceof DynamicControlVelocity velocity) {
            return new DynamicControlVelocity(velocity.control(), velocity.velocitySupplier(),
                    merge(velocity.policy(), update), velocity.effort());
        }
        throw new IllegalArgumentException("Retarget and release policies apply to one position or velocity control"
                + " action. Set the binding default with ControlBinding.retarget(...) or ControlBinding.release(...).");
    }

    static MotionPolicy policyOf(Action action) {
        if (action instanceof ControlPosition position) return position.policy();
        if (action instanceof DynamicControlPosition position) return position.policy();
        if (action instanceof ControlVelocity velocity) return velocity.policy();
        if (action instanceof DynamicControlVelocity velocity) return velocity.policy();
        return MotionPolicy.INHERIT;
    }

    private static MotionPolicy merge(MotionPolicy current, MotionPolicy update) {
        MotionPolicy base = current == null ? MotionPolicy.INHERIT : current;
        return new MotionPolicy(
                update.retarget() != null ? update.retarget() : base.retarget(),
                update.release() != null ? update.release() : base.release());
    }

    /**
     * Starts a fallback chain: each attempt must complete before its time limit to count as a success;
     * a timed-out or missing attempt moves to the next. Finish with {@link Attempt#orElse(Action)} or
     * {@link Attempt#toAction()}.
     */
    public static Attempt attempt(Action first) {
        return new Attempt(List.of(new Attempt.Step(Objects.requireNonNull(first, "first"), Double.POSITIVE_INFINITY)), null);
    }

    /** Fallback chain builder. Each step's time limit applies to the step immediately before it. */
    public static final class Attempt {
        private final List<Step> steps;
        private final Action fallback;

        private Attempt(List<Step> steps, Action fallback) {
            this.steps = steps;
            this.fallback = fallback;
        }

        /** Sets the time limit of the most recently added attempt. */
        public Attempt within(Time duration) {
            double seconds = duration.in(Seconds);
            if (!(seconds > 0.0) || Double.isNaN(seconds)) {
                throw new IllegalArgumentException("Attempt time limit must be positive.");
            }
            List<Step> updated = new ArrayList<>(steps);
            Step last = updated.remove(updated.size() - 1);
            updated.add(new Step(last.action(), seconds));
            return new Attempt(List.copyOf(updated), fallback);
        }

        /** Adds the next attempt, tried when the previous one fails. */
        public Attempt orTry(Action next) {
            Objects.requireNonNull(next, "next");
            List<Step> updated = new ArrayList<>(steps);
            updated.add(new Step(next, Double.POSITIVE_INFINITY));
            return new Attempt(List.copyOf(updated), fallback);
        }

        /** Completes the chain with an Action that runs when every attempt failed. */
        public Action orElse(Action fallbackAction) {
            return new Attempt(steps, Objects.requireNonNull(fallbackAction, "fallbackAction")).toAction();
        }

        /** Returns the chain as an Action; when every attempt fails it completes. */
        public Action toAction() {
            // Built back to front: attempt i runs, then either completes (its flag was marked) or
            // continues with attempt i+1, or with the fallback after the last attempt.
            Action chain = fallback;
            for (int index = steps.size() - 1; index >= 0; index--) {
                Step step = steps.get(index);
                Flag flag = new Flag();
                Action attemptAction = new Then(new DoOnce(flag::clear), timedAttempt(step, flag));
                Action skip = chain == null ? new DoOnce(() -> { }) : chain;
                chain = new Then(attemptAction, new Choice(ctx -> flag.set, new DoOnce(() -> { }), skip));
            }
            return chain;
        }

        private static Action timedAttempt(Step step, Flag flag) {
            Action body = new Then(step.action(), new DoOnce(flag::mark));
            return Double.isFinite(step.seconds()) ? new Timeout(body, Seconds.of(step.seconds())) : body;
        }

        record Step(Action action, double seconds) {
        }

        private static final class Flag {
            private boolean set;

            void mark() { set = true; }

            void clear() { set = false; }
        }
    }

    /** Appends {@code next} after an existing follow-up, keeping every stage already chained. */
    static Action chain(Action existing, Action next) {
        Objects.requireNonNull(next, "next");
        return existing == null ? next : then(existing, next);
    }

    public record Neutral() implements Action, Output.Neutral {
    }

    public record ControlNeutral(ControlBinding<?> control) implements Action, Output.Neutral {
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

    public record ControlPercent(ControlBinding<?> control, double percent, EffortRequest effort) implements Action, Output.Percent, EffortCarrier {
        @Override
        public String describe() {
            return "PERCENT " + percent + effortLabel(effort);
        }

        /** Source-compatible constructor: the Action sets no effort limit. */
        public ControlPercent(ControlBinding<?> control, double percent) {
            this(control, percent, EffortRequest.NONE);
        }

        public ControlPercent {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            requireControlOutput(control);
            requireFinite(percent, "Control percent");
        }
    }

    public record DynamicControlPercent(ControlBinding<?> control, DoubleSupplier percentSupplier, EffortRequest effort) implements Action, Output.Percent, EffortCarrier {
        @Override
        public String describe() {
            return "PERCENT (dynamic)" + effortLabel(effort);
        }

        /** Source-compatible constructor: the Action sets no effort limit. */
        public DynamicControlPercent(ControlBinding<?> control, DoubleSupplier percentSupplier) {
            this(control, percentSupplier, EffortRequest.NONE);
        }

        public DynamicControlPercent {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(percentSupplier, "percentSupplier");
            requireControlOutput(control);
        }

        @Override
        public double percent() {
            return requireFinite(percentSupplier.getAsDouble(), "Control percent");
        }
    }

    public record ControlVoltage(ControlBinding<?> control, double volts, EffortRequest effort) implements Action, Output.Voltage, EffortCarrier {
        @Override
        public String describe() {
            return "VOLTAGE " + volts + effortLabel(effort);
        }

        /** Source-compatible constructor: the Action sets no effort limit. */
        public ControlVoltage(ControlBinding<?> control, double volts) {
            this(control, volts, EffortRequest.NONE);
        }

        public ControlVoltage {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            requireControlOutput(control);
            requireFinite(volts, "Control voltage");
        }
    }

    public record DynamicControlVoltage(ControlBinding<?> control, DoubleSupplier voltsSupplier, EffortRequest effort) implements Action, Output.Voltage, EffortCarrier {
        @Override
        public String describe() {
            return "VOLTAGE (dynamic)" + effortLabel(effort);
        }

        /** Source-compatible constructor: the Action sets no effort limit. */
        public DynamicControlVoltage(ControlBinding<?> control, DoubleSupplier voltsSupplier) {
            this(control, voltsSupplier, EffortRequest.NONE);
        }

        public DynamicControlVoltage {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(voltsSupplier, "voltsSupplier");
            requireControlOutput(control);
        }

        @Override
        public double volts() {
            return requireFinite(voltsSupplier.getAsDouble(), "Control voltage");
        }
    }

    public record ControlPosition(ControlBinding<?> control, double position, MotionPolicy policy, EffortRequest effort)
            implements Action, Output.Position, EffortCarrier {
        @Override
        public String describe() {
            return "POSITION " + position + effortLabel(effort);
        }

        /** Source-compatible constructor: inherited motion policy and no effort limit. */
        public ControlPosition(ControlBinding<?> control, double position) {
            this(control, position, MotionPolicy.INHERIT, EffortRequest.NONE);
        }

        public ControlPosition(ControlBinding<?> control, double position, MotionPolicy policy) {
            this(control, position, policy, EffortRequest.NONE);
        }

        public ControlPosition(ControlBinding<?> control, double position, EffortRequest effort) {
            this(control, position, MotionPolicy.INHERIT, effort);
        }

        public ControlPosition {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            requireControlMode(control, ControlMode.POSITION);
            requireFinite(position, "Control position");
            policy = policy == null ? MotionPolicy.INHERIT : policy;
        }
    }

    public record DynamicControlPosition(ControlBinding<?> control, DoubleSupplier positionSupplier, MotionPolicy policy, EffortRequest effort)
            implements Action, Output.Position, EffortCarrier {
        @Override
        public String describe() {
            return "POSITION (dynamic)" + effortLabel(effort);
        }

        /** Source-compatible constructor: inherited motion policy and no effort limit. */
        public DynamicControlPosition(ControlBinding<?> control, DoubleSupplier positionSupplier) {
            this(control, positionSupplier, MotionPolicy.INHERIT, EffortRequest.NONE);
        }

        public DynamicControlPosition(ControlBinding<?> control, DoubleSupplier positionSupplier, MotionPolicy policy) {
            this(control, positionSupplier, policy, EffortRequest.NONE);
        }

        public DynamicControlPosition(ControlBinding<?> control, DoubleSupplier positionSupplier, EffortRequest effort) {
            this(control, positionSupplier, MotionPolicy.INHERIT, effort);
        }

        public DynamicControlPosition {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(positionSupplier, "positionSupplier");
            requireControlMode(control, ControlMode.POSITION);
            policy = policy == null ? MotionPolicy.INHERIT : policy;
        }

        @Override
        public double position() {
            return requireFinite(positionSupplier.getAsDouble(), "Control position");
        }
    }

    public record ControlVelocity(ControlBinding<?> control, double velocity, MotionPolicy policy, EffortRequest effort)
            implements Action, Output.Velocity, EffortCarrier {
        @Override
        public String describe() {
            return "VELOCITY " + velocity + effortLabel(effort);
        }

        /** Source-compatible constructor: inherited motion policy and no effort limit. */
        public ControlVelocity(ControlBinding<?> control, double velocity) {
            this(control, velocity, MotionPolicy.INHERIT, EffortRequest.NONE);
        }

        public ControlVelocity(ControlBinding<?> control, double velocity, MotionPolicy policy) {
            this(control, velocity, policy, EffortRequest.NONE);
        }

        public ControlVelocity(ControlBinding<?> control, double velocity, EffortRequest effort) {
            this(control, velocity, MotionPolicy.INHERIT, effort);
        }

        public ControlVelocity {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            requireControlMode(control, ControlMode.VELOCITY);
            requireFinite(velocity, "Control velocity");
            policy = policy == null ? MotionPolicy.INHERIT : policy;
        }
    }

    public record DynamicControlVelocity(ControlBinding<?> control, DoubleSupplier velocitySupplier, MotionPolicy policy, EffortRequest effort)
            implements Action, Output.Velocity, EffortCarrier {
        @Override
        public String describe() {
            return "VELOCITY (dynamic)" + effortLabel(effort);
        }

        /** Source-compatible constructor: inherited motion policy and no effort limit. */
        public DynamicControlVelocity(ControlBinding<?> control, DoubleSupplier velocitySupplier) {
            this(control, velocitySupplier, MotionPolicy.INHERIT, EffortRequest.NONE);
        }

        public DynamicControlVelocity(ControlBinding<?> control, DoubleSupplier velocitySupplier, MotionPolicy policy) {
            this(control, velocitySupplier, policy, EffortRequest.NONE);
        }

        public DynamicControlVelocity(ControlBinding<?> control, DoubleSupplier velocitySupplier, EffortRequest effort) {
            this(control, velocitySupplier, MotionPolicy.INHERIT, effort);
        }

        public DynamicControlVelocity {
            effort = effort == null ? EffortRequest.NONE : effort;
            Objects.requireNonNull(control, "control");
            Objects.requireNonNull(velocitySupplier, "velocitySupplier");
            requireControlMode(control, ControlMode.VELOCITY);
            policy = policy == null ? MotionPolicy.INHERIT : policy;
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
            context.encoder(encoder).setPositionRotations(encoder.rotationsFromPosition(encoder.measure(position)));
        }
    }

    public interface ImuYawMutation extends Action {
        ImuSource imu();

        double requestedYawDegrees();

        @Override
        default void apply(ActionContext context) {
            imu().applyYawDegrees(context, requestedYawDegrees());
        }
    }

    public record VelocityContribution(
            RobotVelocityPool.Channel channel,
            Supplier<RobotVelocity> velocity,
            Action driveAction) implements Action {
        @Override
        public List<Child> children() {
            return childrenOf(child("drive", driveAction));
        }

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

    public record WaitSeconds(Time duration) implements Action {
        public WaitSeconds {
            requireDuration(duration.in(Seconds), "Wait duration");
        }

        @Override
        public String describe() {
            return "WAIT " + duration.in(Seconds) + "s";
        }

        public boolean complete(MechanismContext ctx) {
            return hasElapsed(ctx.timeInStateSeconds(), duration.in(Seconds));
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

    public record WaitUntilStalled(MotorStallSignal signal) implements Action {
        public WaitUntilStalled {
            Objects.requireNonNull(signal, "signal");
        }
    }

    /** One-shot branch: the condition is sampled when the action starts. A null inactive branch skips. */
    public record Either(ActionCondition condition, Action active, Action inactive) implements Action {
        public Either {
            Objects.requireNonNull(condition, "condition");
            Objects.requireNonNull(active, "active");
        }

        public Action choose(MechanismContext ctx) {
            return condition.test(ctx) ? active : inactive;
        }

        @Override
        public List<Child> children() {
            return childrenOf(child("true", active), child("false", inactive));
        }

        @Override
        public String describe() {
            return inactive == null ? "ONLY IF" : "EITHER";
        }
    }

    public record Timeout(Action action, Time duration) implements Action {

        @Override
        public List<Child> children() {
            return childrenOf(child("body", action));
        }

        @Override
        public String describe() {
            return "TIMEOUT " + duration.in(Seconds) + "s";
        }

        public Timeout {
            Objects.requireNonNull(action, "action");
            requireDuration(duration.in(Seconds), "Timeout duration");
        }

        public boolean expired(MechanismContext ctx) {
            return hasElapsed(ctx.timeInStateSeconds(), duration.in(Seconds));
        }
    }

    public record Choice(ActionCondition condition, Action active, Action inactive) implements Action {

        @Override
        public List<Child> children() {
            return childrenOf(child("true", active), child("false", inactive));
        }

        @Override
        public String describe() {
            return "CONDITIONAL";
        }

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

        @Override
        public List<Child> children() {
            return childrenOf(child("true", active));
        }

        @Override
        public String describe() {
            return "CONDITIONAL";
        }

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

        @Override
        public List<Child> children() {
            List<Child> out = new ArrayList<>();
            addChildren(out, "branch", Actions());
            return List.copyOf(out);
        }

        @Override
        public String describe() {
            return "PARALLEL";
        }

        public Parallel {
            Actions = copyStates(Actions);
        }
    }

    public record Race(List<Action> Actions) implements Action {

        @Override
        public List<Child> children() {
            List<Child> out = new ArrayList<>();
            addChildren(out, "branch", Actions());
            return List.copyOf(out);
        }

        @Override
        public String describe() {
            return "RACE";
        }

        public Race {
            Actions = copyStates(Actions);
        }
    }

    public static final class Deadline implements Action {

        @Override
        public List<Child> children() {
            List<Child> out = new ArrayList<>();
            out.add(new Child("primary", primary));
            addChildren(out, "with", others);
            return List.copyOf(out);
        }

        @Override
        public String describe() {
            return "DEADLINE";
        }

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

    /**
     * Runs {@code action} until {@code condition} holds, then {@code next} when one is chained. A second
     * chained continuation sequences after the first instead of replacing it.
     */
    public record Conditional(Action action, ActionCondition condition, Action next) implements Action {
        public Conditional {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(condition, "condition");
        }

        @Override
        public Action then(Action continuation) {
            Objects.requireNonNull(continuation, "next");
            return next == null ? new Conditional(action, condition, continuation) : new Then(this, continuation);
        }

        @Override
        public List<Child> children() {
            return childrenOf(child("body", action), child("then", next));
        }

        @Override
        public String describe() {
            return "UNTIL";
        }
    }

    /**
     * A control action that completes once its measured feedback is inside {@code tolerance} of the
     * target, held there for {@code settleSeconds} and, when {@code maxVelocity} is finite, moving no
     * faster than {@code maxVelocity}. The defaults ({@code settleSeconds == 0}, no velocity limit) keep
     * the original {@code untilWithin(tolerance)} behaviour, which completes on the first in-window cycle.
     *
     * @param action control action being measured
     * @param tolerance maximum absolute target error in the control's units
     * @param settleSeconds continuous time the measurement must stay inside the window before completing
     * @param maxVelocity maximum absolute measured velocity (control units per second); infinity for none
     */
    public record WithinTolerance(Action action, double tolerance, double settleSeconds, double maxVelocity)
            implements Action {
        @Override
        public List<Child> children() {
            return childrenOf(child("body", action));
        }

        @Override
        public String describe() {
            StringBuilder label = new StringBuilder("WITHIN ").append(tolerance);
            if (settleSeconds > 0.0) label.append(" settled ").append(settleSeconds).append("s");
            if (Double.isFinite(maxVelocity)) label.append(" maxVelocity ").append(maxVelocity);
            return label.toString();
        }

        public WithinTolerance(Action action, double tolerance) {
            this(action, tolerance, 0.0, Double.POSITIVE_INFINITY);
        }

        public WithinTolerance {
            Objects.requireNonNull(action, "action");
            if (!supportsTolerance(action)) {
                throw new IllegalArgumentException(
                        "untilWithin requires one position or velocity control action.");
            }
            if (!Double.isFinite(tolerance) || tolerance < 0.0) {
                throw new IllegalArgumentException("Control tolerance must be finite and non-negative.");
            }
            if (!Double.isFinite(settleSeconds) || settleSeconds < 0.0) {
                throw new IllegalArgumentException("Settle time must be finite and non-negative.");
            }
            if (Double.isNaN(maxVelocity) || maxVelocity < 0.0) {
                throw new IllegalArgumentException("Maximum velocity must be non-negative.");
            }
            if (Double.isFinite(maxVelocity) && isVelocityControl(action)) {
                throw new IllegalArgumentException(
                        "maxVelocity applies to position controls only; a velocity control already targets a velocity.");
            }
        }
    }

    /** Runs {@code action} to completion, then {@code next}. Pure sequencing glue: no label of its own. */
    public record Then(Action action, Action next) implements Action {
        public Then {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(next, "next");
        }

        @Override
        public List<Child> children() {
            return childrenOf(child("first", action), child("then", next));
        }

        @Override
        public String describe() {
            return "";
        }
    }

    /**
     * Ordered stages. {@link #then(Action)} and {@link #timeout(double)} always return a new
     * sequence, like every other action composition, so {@code intake.then(stop)} never changes
     * {@code intake}. The other builder calls mutate until the sequence is first requested or
     * declared on a registered mechanism and return a modified copy after that.
     */
    @SuppressWarnings("unchecked") // Action.timeout(Time) overrides DeviceAction's generic timeout.
    public static final class Sequence implements Action {

        @Override
        public List<Child> children() {
            List<Child> out = new ArrayList<>();
            for (SequenceStep step : steps) {
                out.add(new Child("step", step.action()));
            }
            Child continuation = child("then", next);
            if (continuation != null) out.add(continuation);
            return List.copyOf(out);
        }

        @Override
        public String describe() {
            return Double.isFinite(timeoutSeconds) ? "SEQUENCE TIMEOUT " + timeoutSeconds + "s" : "SEQUENCE";
        }

        private final List<SequenceStep> steps = new ArrayList<>();
        private final List<SequenceStep> stepView = Collections.unmodifiableList(steps);
        private final List<Action> started = new ArrayList<>();
        private final Set<SequenceStep> timedSteps = Collections.newSetFromMap(new IdentityHashMap<>());
        private Action next;
        private double timeoutSeconds = Double.POSITIVE_INFINITY;
        private boolean sealed;

        private Sequence() {
        }

        public Sequence run(Action action) {
            Sequence target = mutable();
            target.steps.add(new SequenceStep(target.withStartedUntilComplete(action), ctx -> false, false));
            return target;
        }

        /** Declaration overload of {@link #run(Action)}, for {@code encoder.setPosition(...)} and similar. */
        public Sequence run(DeviceAction action) {
            return run(mechanismAction(action));
        }

        public Sequence forTime(Time duration, Action action) {
            double seconds = requireDuration(duration.in(Seconds), "Sequence stage duration");
            Sequence target = mutable();
            SequenceStep step = new SequenceStep(
                    target.alongsideStarted(action),
                    ctx -> hasElapsed(ctx.timeInStateSeconds(), seconds),
                    true);
            target.steps.add(step);
            target.timedSteps.add(step);
            return target;
        }

        /** True when the step's completion is bounded by its own time limit (forTime). */
        boolean boundedStep(SequenceStep step) {
            return timedSteps.contains(step);
        }

        public Sequence until(ActionCondition condition, Action action) {
            Objects.requireNonNull(condition, "condition");
            Sequence target = mutable();
            target.steps.add(new SequenceStep(target.alongsideStarted(action), condition, true));
            return target;
        }

        public Sequence until(BooleanSupplier condition, Action action) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), action);
        }

        /** Runs {@code action} until {@code signal} detects a stall, then advances. */
        public Sequence untilStalled(MotorStallSignal signal, Action action) {
            return run(Actions.untilStalled(signal, action));
        }

        public Sequence doOnce(Runnable action) {
            return addOnce(Actions.doOnce(action));
        }

        public Sequence doOnce(ActionBinding action) {
            return addOnce(Actions.doOnce(action));
        }

        /** Scheduled as the Action itself, so its leases and ownership apply within this sequence. */
        public Sequence doOnce(DeviceAction action) {
            return addOnce(Actions.doOnce(action));
        }

        private Sequence addOnce(Action once) {
            Sequence target = mutable();
            target.steps.add(new SequenceStep(target.withStartedUntilComplete(once), ctx -> true, true));
            return target;
        }

        public Sequence waitSeconds(Time duration) {
            double seconds = duration.in(Seconds);
            Action wait = Actions.waitSeconds(duration);
            Sequence target = mutable();
            target.steps.add(new SequenceStep(
                    target.alongsideStarted(wait),
                    ctx -> hasElapsed(ctx.timeInStateSeconds(), seconds),
                    true));
            return target;
        }

        public Sequence waitUntil(ActionCondition condition) {
            Sequence target = mutable();
            target.steps.add(new SequenceStep(target.alongsideStarted(Actions.waitUntil(condition)), condition, true));
            return target;
        }

        public Sequence waitUntil(BooleanSupplier condition) {
            Objects.requireNonNull(condition, "condition");
            return waitUntil(ctx -> condition.getAsBoolean());
        }

        /**
         * Limits everything built so far to {@code seconds}. Stages added afterwards, including
         * {@code then(...)}, run after the limited part, as with {@code timeout(...)} on any action.
         */
        @Override
        public Sequence timeout(Time duration) {
            requireDuration(duration.in(Seconds), "Sequence timeout");
            Sequence outer = new Sequence();
            outer.steps.add(new SequenceStep(new Timeout(copy(), duration), ctx -> false, false));
            return outer;
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
            Sequence target = mutable();
            target.started.add(safeAction);
            return target;
        }

        /** Stops carrying a previously started action into subsequent stages. */
        public Sequence stop(Action action) {
            Action safeAction = Objects.requireNonNull(action, "action");
            Sequence target = mutable();
            target.started.removeIf(existing -> existing == safeAction);
            return target;
        }

        @Override
        public Sequence then(Action action) {
            Sequence target = copy();
            Action safeAction = target.withStartedUntilComplete(Objects.requireNonNull(action, "action"));
            target.next = target.next == null ? safeAction : Actions.then(target.next, safeAction);
            return target;
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

        void seal() {
            sealed = true;
        }

        private Sequence mutable() {
            return sealed ? copy() : this;
        }

        private Sequence copy() {
            Sequence copy = new Sequence();
            copy.steps.addAll(steps);
            copy.started.addAll(started);
            copy.timedSteps.addAll(timedSteps);
            copy.next = next;
            copy.timeoutSeconds = timeoutSeconds;
            return copy;
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

    /** Repeating stages. Like {@link Sequence}, builder calls copy once the cycle is requested or declared. */
    public static final class Cycle implements Action {

        @Override
        public List<Child> children() {
            List<Child> out = new ArrayList<>();
            for (CycleStep step : steps) {
                out.add(new Child("step", step.action()));
            }
            return List.copyOf(out);
        }

        @Override
        public String describe() {
            return "CYCLE";
        }

        private final List<CycleStep> steps = new ArrayList<>();
        private final List<CycleStep> stepView = Collections.unmodifiableList(steps);
        private boolean sealed;

        private Cycle() {
        }

        public Cycle run(Action action) {
            return add(new CycleStep(action, ctx -> false));
        }

        public Cycle forTime(Time duration, Action action) {
            double seconds = requireDuration(duration.in(Seconds), "Cycle stage duration");
            return add(new CycleStep(action, ctx -> hasElapsed(ctx.timeInStateSeconds(), seconds)));
        }

        public Cycle until(ActionCondition condition, Action action) {
            return add(new CycleStep(action, condition));
        }

        public Cycle until(BooleanSupplier condition, Action action) {
            Objects.requireNonNull(condition, "condition");
            return until(ctx -> condition.getAsBoolean(), action);
        }

        /** Runs {@code action} until {@code signal} detects a stall, then advances. */
        public Cycle untilStalled(MotorStallSignal signal, Action action) {
            return run(Actions.untilStalled(signal, action));
        }

        public List<CycleStep> steps() {
            return stepView;
        }

        void seal() {
            sealed = true;
        }

        private Cycle add(CycleStep step) {
            Cycle target = this;
            if (sealed) {
                target = new Cycle();
                target.steps.addAll(steps);
            }
            target.steps.add(step);
            return target;
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
        rejectNestedRootOnly(action);
        if (action instanceof Sequence sequence) {
            sequence.seal();
            validateSequence(sequence);
        } else if (action instanceof Then then) {
            validateThenSource(then.action());
        }
        for (Action.Child child : action.children()) {
            validate(child.action(), visited);
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
                || action instanceof WaitUntilStalled
                || action instanceof Timeout || action instanceof WithinTolerance
                || action instanceof Conditional) {
            return Completion.POSSIBLE;
        }
        if (action instanceof LifecycleAction lifecycle) {
            return lifecycle.finishes() ? Completion.POSSIBLE : Completion.NEVER;
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
        if (action instanceof VelocityContribution contribution) {
            return completion(contribution.driveAction(), visited);
        }
        if (action instanceof Either either) {
            if (either.inactive() == null) return Completion.POSSIBLE;
            Completion active = completion(either.active(), visited);
            Completion inactive = completion(either.inactive(), visited);
            if (active == Completion.POSSIBLE && inactive == Completion.POSSIBLE) return Completion.POSSIBLE;
            return active == Completion.NEVER && inactive == Completion.NEVER ? Completion.NEVER : Completion.UNKNOWN;
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

    private static void requireControlOutput(ControlBinding<?> control) {
        if (control.output() == null && control.sink() == null) {
            throw new IllegalStateException("Control actions require an output destination.");
        }
    }

    private static void requireControlMode(ControlBinding<?> control, ControlMode required) {
        requireControlOutput(control);
        if (control.mode() != required) {
            throw new IllegalStateException(required.name().toLowerCase(java.util.Locale.ROOT)
                    + " action requires a matching control binding.");
        }
    }

    private static double requireDuration(double seconds, String description) {
        if (Double.isNaN(seconds)) {
            throw new IllegalArgumentException(description + " must be a number of seconds, not NaN.");
        }
        return seconds;
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
