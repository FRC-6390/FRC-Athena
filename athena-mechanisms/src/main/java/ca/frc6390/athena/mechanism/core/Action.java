package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;
import ca.frc6390.athena.mechanism.effort.EffortActions;
import ca.frc6390.athena.mechanism.staging.Staging;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Robot intent for a mechanism.
 */
// The overrides of DeviceAction's generic <T> then/untilStalled return Action, which is a T by the
// DeviceAction bound; javac reports the erasure as unchecked at the override, so it is suppressed here.
@SuppressWarnings("unchecked")
public interface Action extends DeviceAction {
    @Override
    default void apply(ActionContext context) {
        request();
    }

    /**
     * Structural children of this Action in declaration order. Read-only walkers (auto plan listing,
     * path discovery, ownership and resource scans) traverse the Action tree through this method, so a
     * new composite implements it once instead of being added to every instanceof chain. Leaves have no
     * children. Null children are never reported.
     *
     * @return labelled children; empty for leaves
     */
    default List<Child> children() {
        return List.of();
    }

    /**
     * Short human label for this node, such as {@code TIMEOUT 2.0s} or {@code WITHIN 1.5 settled 0.1s}.
     * Pure sequencing glue returns an empty label and has no line of its own. Defaults to the simple
     * class name.
     *
     * @return one-line label
     */
    default String describe() {
        String name = getClass().getSimpleName();
        return name == null || name.isBlank() ? getClass().getName() : name;
    }

    /**
     * A structural child of an Action.
     *
     * @param role how the parent uses the child: {@code step}, {@code branch}, {@code body}, {@code first},
     *             {@code then}, {@code primary}, {@code with}, {@code true}, {@code false}, {@code drive},
     *             or {@code marker:<name>} for a path marker
     * @param action the child Action
     */
    record Child(String role, Action action) {
        public Child {
            Objects.requireNonNull(role, "role");
            Objects.requireNonNull(action, "action");
        }
    }

    /**
     * Requests this action through the currently bound robot runtime. Re-requesting the
     * same instance restarts it and gives its resource lease newest-request precedence.
     *
     * @throws IllegalStateException when no robot runtime is bound
     * @throws IllegalArgumentException when the runtime cannot infer mechanism ownership
     */
    default void request() {
        ActionRequests.request(this);
    }

    /**
     * Makes a declared {@link Staging} active only while this Action runs: its conditions are evaluated and
     * its caps apply only during that time. Returns this Action unchanged so it can be used as a field.
     *
     * @param staging declared staging
     * @return this action
     */
    default Action staged(Staging<?> staging) {
        Objects.requireNonNull(staging, "staging").gate(this);
        return this;
    }

    default Action until(ActionCondition condition) {
        return Actions.until(condition, this);
    }

    default Action until(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return until(ctx -> condition.getAsBoolean());
    }

    /**
     * Limits the current this control Action may draw while it runs (B5). The value is a torque, force or
     * current measure of the control's unit kind: rotary controls accept Torque or Current, linear controls
     * Force or Current, and untyped controls Current only. Torque and force convert through the binding's
     * declared gearing and the motors' constants. The limit is applied as a voltage clamp (B6), so it works
     * with any controller. Composite Actions pass the limit to their control Actions.
     *
     * @param limit maximum effort
     * @return Action with the limit
     */
    default Action effort(Measure<?> limit) {
        return EffortActions.limit(this, limit);
    }

    /**
     * Completes this position or velocity control action when its feedback is within the tolerance of the
     * requested target. The tolerance is a WPILib measure of the control's unit (a rate of that unit for
     * velocity controls). Requires a unit-typed control binding.
     *
     * @param tolerance maximum absolute target error as a measure
     * @return feedback-completing action
     */
    default Action untilWithin(Measure<?> tolerance) {
        return Actions.untilWithin(this, tolerance);
    }

    /**
     * Requires the measurement of an {@link #untilWithin(Measure) untilWithin} action to stay inside its
     * window continuously for {@code seconds} before it completes. The timer restarts whenever the
     * measurement leaves the window. Use this to stop a mechanism that swings through the target at speed
     * from completing on the first in-window cycle and coasting past it.
     *
     * @param duration required continuous time inside the window (finite, non-negative)
     * @return settled tolerance action
     * @throws IllegalArgumentException when this action is not an {@code untilWithin} action
     */
    default Action settled(Time duration) {
        return Actions.settled(this, Objects.requireNonNull(duration, "duration"));
    }

    /** {@link #settled(Time)} for a {@link java.time.Duration}. */
    default Action settled(java.time.Duration duration) {
        Objects.requireNonNull(duration, "duration");
        return Actions.settled(this, Seconds.of(duration.toNanos() / 1.0e9));
    }

    /**
     * Requires the measured velocity of an {@link #untilWithin(Measure) untilWithin} position action to be
     * at most {@code maxVelocity} in magnitude for it to complete. The limit is a rate of the control
     * binding's unit per second. Velocity controls are rejected: their target is already a velocity.
     *
     * @param maxVelocity maximum absolute measured velocity, non-negative
     * @return velocity-limited tolerance action
     * @throws IllegalArgumentException when this action is not a position {@code untilWithin} action
     */
    default Action maxVelocity(Measure<?> maxVelocity) {
        return Actions.maxVelocity(this, maxVelocity);
    }

    /**
     * Convenience for {@code untilWithin(tolerance).settled(duration)}: completes once the position or
     * velocity error stays within {@code tolerance} continuously for {@code duration}.
     *
     * @param tolerance maximum absolute target error as a measure of the control's unit
     * @param duration required continuous time inside the window
     * @return settled tolerance action
     */
    default Action untilSettled(Measure<?> tolerance, Time duration) {
        return untilWithin(tolerance).settled(duration);
    }

    /** {@link #untilSettled(Measure, Time)} for a {@link java.time.Duration}. */
    default Action untilSettled(Measure<?> tolerance, java.time.Duration settle) {
        return untilWithin(tolerance).settled(settle);
    }

    default Action then(Action next) {
        return Actions.then(this, next);
    }

    /**
     * Runs a hardware declaration action (for example {@code encoder.setPosition(0.0)} or
     * {@code motor.neutral()}) after this Action and returns the composed Action. Overrides the generic
     * {@code DeviceAction.then} so the result stays an {@code Action} and chains with Action methods.
     */
    @Override
    default Action then(DeviceAction next) {
        return Actions.then(this, Objects.requireNonNull(next, "next"));
    }

    /**
     * Runs this Action until the stall detector reports a commanded motor that is not moving. Overrides
     * the generic {@code DeviceAction.untilStalled} so the result stays an {@code Action}.
     */
    @Override
    default Action untilStalled(MotorStallSignal signal) {
        return Actions.untilStalled(signal, this);
    }

    default Action timeout(Time duration) {
        return Actions.timeout(this, Objects.requireNonNull(duration, "duration"));
    }

    /**
     * Bounds this action to the supplied duration, then completes it. Required to make a continuous
     * action eligible for {@link #interruptible(Interrupt) AFTER_COMPLETE} policies.
     */
    default Action atMost(java.time.Duration duration) {
        Objects.requireNonNull(duration, "duration");
        return Actions.timeout(this, Seconds.of(duration.toNanos() / 1.0e9));
    }

    /**
     * Overrides how a new target re-plans a running profile for this position or velocity control
     * action. Binding default: {@link ControlBinding#retarget(Retarget)}.
     *
     * @throws IllegalArgumentException when this action is not a single position or velocity control
     */
    default Action retarget(Retarget retarget) {
        return Actions.withMotionPolicy(this, new MotionPolicy(Objects.requireNonNull(retarget, "retarget"), null));
    }

    /**
     * Overrides what happens to the binding when this action is the last one driving it. Binding
     * default: {@link ControlBinding#release(Release)}.
     *
     * @throws IllegalArgumentException when this action is not a single position or velocity control
     */
    default Action release(Release release) {
        return Actions.withMotionPolicy(this, new MotionPolicy(null, Objects.requireNonNull(release, "release")));
    }

    /**
     * Reaches a new velocity target in {@code time}, no sooner, for this velocity control action: the speed
     * reference ramps at the constant acceleration that arrives exactly then (never above the binding's own
     * acceleration limit). Use it when the speed is only needed at a known moment, such as a flywheel that has to
     * be ready when the aim settles. For a given change of speed the heat lost in the motor grows with the current
     * used, so taking the time that is available instead of the fastest possible spin-up costs less battery and
     * less heat. A new target starts a new ramp from the current reference.
     *
     * @param time time to reach each new target, positive
     * @throws IllegalArgumentException when this action is not a single position or velocity control
     */
    default Action arriveWithin(edu.wpi.first.units.measure.Time time) {
        double seconds = Objects.requireNonNull(time, "time").in(Seconds);
        return Actions.withMotionPolicy(this, new MotionPolicy(null, null, seconds));
    }

    /**
     * Sets how a newer request for overlapping resources may replace this running action. Only valid
     * on a root action (the one passed to {@link #request()}). Restrictive policies require a bounded
     * action; see {@link Actions#isBounded(Action)}.
     *
     * @throws IllegalArgumentException when the action is unbounded for a restrictive policy
     */
    default Action interruptible(Interrupt interrupt) {
        return Actions.interruptible(this, Objects.requireNonNull(interrupt, "interrupt"));
    }

}
