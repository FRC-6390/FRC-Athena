package ca.frc6390.athena.mechanism.effort;

import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import edu.wpi.first.units.Measure;
import java.util.Objects;

/**
 * Applies a per-Action effort limit (B5) to control Actions.
 *
 * <p>The limit is converted to a per-motor current when the Action is declared, so the control path
 * only compares doubles. Torque and force need the binding's gearing, which is declared with
 * {@code Effort.max(...).gearing(...)} on the binding before its Actions are created.</p>
 */
public final class EffortActions {
    private EffortActions() {
    }

    /**
     * Returns the Action with an effort limit while it runs. Composite Actions (conditionals, {@code then}
     * chains, timeouts, tolerance wrappers) carry the limit to their control Actions.
     *
     * @param action control or composite Action
     * @param limit maximum torque, force or current; must suit the binding's motion kind
     * @return Action with the limit
     * @throws IllegalArgumentException when the unit does not suit the control, or the Action has no control
     * @throws IllegalArgumentException when torque or force is requested but the binding has no gearing
     */
    public static Action limit(Action action, Measure<?> limit) {
        Objects.requireNonNull(action, "action");
        Objects.requireNonNull(limit, "limit");
        if (action instanceof Actions.ControlPosition a) {
            return new Actions.ControlPosition(a.control(), a.position(), request(a.control(), limit));
        }
        if (action instanceof Actions.DynamicControlPosition a) {
            return new Actions.DynamicControlPosition(a.control(), a.positionSupplier(), request(a.control(), limit));
        }
        if (action instanceof Actions.ControlVelocity a) {
            return new Actions.ControlVelocity(a.control(), a.velocity(), request(a.control(), limit));
        }
        if (action instanceof Actions.DynamicControlVelocity a) {
            return new Actions.DynamicControlVelocity(a.control(), a.velocitySupplier(), request(a.control(), limit));
        }
        if (action instanceof Actions.ControlPercent a) {
            return new Actions.ControlPercent(a.control(), a.percent(), request(a.control(), limit));
        }
        if (action instanceof Actions.DynamicControlPercent a) {
            return new Actions.DynamicControlPercent(a.control(), a.percentSupplier(), request(a.control(), limit));
        }
        if (action instanceof Actions.ControlVoltage a) {
            return new Actions.ControlVoltage(a.control(), a.volts(), request(a.control(), limit));
        }
        if (action instanceof Actions.DynamicControlVoltage a) {
            return new Actions.DynamicControlVoltage(a.control(), a.voltsSupplier(), request(a.control(), limit));
        }
        if (action instanceof Action.Conditional a) {
            return new Action.Conditional(limit(a.action(), limit), a.condition(),
                    a.next() == null ? null : limit(a.next(), limit));
        }
        if (action instanceof Action.Then a) {
            return new Action.Then(limit(a.action(), limit), limit(a.next(), limit));
        }
        if (action instanceof Actions.Conditional a) {
            return new Actions.Conditional(limit(a.action(), limit), a.condition(),
                    a.next() == null ? null : limit(a.next(), limit));
        }
        if (action instanceof Actions.Then a) {
            return new Actions.Then(limit(a.action(), limit), limit(a.next(), limit));
        }
        if (action instanceof Actions.Timeout a) {
            return new Actions.Timeout(limit(a.action(), limit), a.seconds());
        }
        if (action instanceof Actions.WithinTolerance a) {
            return new Actions.WithinTolerance(limit(a.action(), limit), a.tolerance());
        }
        throw new IllegalArgumentException("effort(...) applies to control position, velocity, percent and "
                + "voltage Actions, not " + action.getClass().getSimpleName() + ".");
    }

    /**
     * Converts a declared effort into the per-motor limit of one Action. The limit is the smallest
     * per-motor value, so the Action respects every motor of the binding.
     *
     * @param control control the Action drives
     * @param limit declared effort
     * @return request for the Action
     */
    static EffortRequest request(ControlBinding<?> control, Measure<?> limit) {
        Objects.requireNonNull(control, "control");
        MotionKind motion = control.motionKind();
        EffortUnits.Kind kind = EffortUnits.kind(limit, motion);
        EffortLimit declared = declaredLimit(control);
        double gearing = declared == null ? Double.NaN : declared.effort().gearing();
        Measure<?> radius = declared == null ? null : declared.effort().radius();
        if (kind != EffortUnits.Kind.CURRENT && declared == null) {
            throw new IllegalArgumentException((kind == EffortUnits.Kind.TORQUE ? "Torque" : "Force")
                    + " effort requires known gearing — use Amps, or declare the binding's effort with "
                    + "Effort.max(...).gearing(...) before creating its Actions.");
        }
        double[] perMotor = EffortUnits.perMotorAmps(limit, kind, control.motors(), gearing, radius);
        double minimum = Double.POSITIVE_INFINITY;
        for (double amps : perMotor) {
            minimum = Math.min(minimum, amps);
        }
        return new EffortRequest(minimum);
    }

    /** Returns the binding's effort declaration, or null when it has none. */
    static EffortLimit declaredLimit(ControlBinding<?> control) {
        for (var loop : control.loops()) {
            if (loop instanceof EffortLimit limit) {
                return limit;
            }
        }
        return null;
    }
}
