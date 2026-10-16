package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.mechanism.effort.EffortActions;
import ca.frc6390.athena.mechanism.staging.Staging;
import edu.wpi.first.units.Measure;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Robot intent for a mechanism.
 */
public interface Action extends DeviceAction {
    @Override
    default void apply(ActionContext context) {
        request();
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
        return new Conditional(this, condition, null);
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
     * Completes this position or velocity control action when its configured
     * feedback is within the supplied tolerance of the requested target.
     *
     * @param tolerance maximum absolute target error in the control's configured units
     * @return feedback-completing action
     */
    default Action untilWithin(double tolerance) {
        return Actions.untilWithin(this, tolerance);
    }

    /**
     * Typed form of {@link #untilWithin(double)}: the tolerance is a WPILib measure of the control's
     * unit (a rate of that unit for velocity controls). Requires a unit-typed control binding.
     *
     * @param tolerance maximum absolute target error as a measure
     * @return feedback-completing action
     */
    default Action untilWithin(Measure<?> tolerance) {
        return Actions.untilWithin(this, tolerance);
    }

    /**
     * Requires the measurement of an {@link #untilWithin(double) untilWithin} action to stay inside its
     * window continuously for {@code seconds} before it completes. The timer restarts whenever the
     * measurement leaves the window. Use this to stop a mechanism that swings through the target at speed
     * from completing on the first in-window cycle and coasting past it.
     *
     * @param seconds required continuous time inside the window (finite, non-negative)
     * @return settled tolerance action
     * @throws IllegalArgumentException when this action is not an {@code untilWithin} action
     */
    default Action settled(double seconds) {
        return Actions.settled(this, seconds);
    }

    /** Typed form of {@link #settled(double)}. */
    default Action settled(java.time.Duration duration) {
        Objects.requireNonNull(duration, "duration");
        return Actions.settled(this, duration.toNanos() / 1.0e9);
    }

    /**
     * Requires the measured velocity of an {@link #untilWithin(double) untilWithin} position action to be
     * at most {@code maxVelocity} in magnitude (control units per second) for it to complete. Velocity
     * controls are rejected: their target is already a velocity.
     *
     * @param maxVelocity maximum absolute measured velocity, non-negative
     * @return velocity-limited tolerance action
     * @throws IllegalArgumentException when this action is not a position {@code untilWithin} action
     */
    default Action maxVelocity(double maxVelocity) {
        return Actions.maxVelocity(this, maxVelocity);
    }

    /** Typed form of {@link #maxVelocity(double)}: a rate of the control binding's unit per second. */
    default Action maxVelocity(Measure<?> maxVelocity) {
        return Actions.maxVelocity(this, maxVelocity);
    }

    /**
     * Convenience for {@code untilWithin(tolerance).settled(seconds)}: completes once the position or
     * velocity error stays within {@code tolerance} continuously for {@code seconds}.
     *
     * @param tolerance maximum absolute target error in the control's configured units
     * @param seconds required continuous time inside the window
     * @return settled tolerance action
     */
    default Action untilSettled(double tolerance, double seconds) {
        return untilWithin(tolerance).settled(seconds);
    }

    /** Typed form of {@link #untilSettled(double, double)}. */
    default Action untilSettled(Measure<?> tolerance, java.time.Duration settle) {
        return untilWithin(tolerance).settled(settle);
    }

    default Action then(Action next) {
        return new Then(this, next);
    }

    default Action timeout(double seconds) {
        return Actions.timeout(this, seconds);
    }

    /**
     * Bounds this action to the supplied duration, then completes it. Required to make a continuous
     * action eligible for {@link #interruptible(Interrupt) AFTER_COMPLETE} policies.
     */
    default Action atMost(java.time.Duration duration) {
        Objects.requireNonNull(duration, "duration");
        return Actions.timeout(this, duration.toNanos() / 1.0e9);
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
     * Sets how a newer request for overlapping resources may replace this running action. Only valid
     * on a root action (the one passed to {@link #request()}). Restrictive policies require a bounded
     * action; see {@link Actions#isBounded(Action)}.
     *
     * @throws IllegalArgumentException when the action is unbounded for a restrictive policy
     */
    default Action interruptible(Interrupt interrupt) {
        return Actions.interruptible(this, Objects.requireNonNull(interrupt, "interrupt"));
    }


    record Conditional(Action action, ActionCondition condition, Action next) implements Action {
        public Conditional {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(condition, "condition");
        }

        @Override
        public Action then(Action next) {
            return new Conditional(action, condition, Objects.requireNonNull(next, "next"));
        }
    }

    record Then(Action action, Action next) implements Action {
        public Then {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(next, "next");
        }
    }

}
