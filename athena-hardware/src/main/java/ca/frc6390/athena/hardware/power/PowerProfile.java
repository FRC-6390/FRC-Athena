package ca.frc6390.athena.hardware.power;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import java.util.Objects;
import java.util.function.UnaryOperator;

/**
 * Shared, mutable electrical budget for one or more motors.
 *
 * <p>A profile is referenced by {@link ca.frc6390.athena.hardware.device.MotorDevice#power(PowerProfile)}.
 * Changing it (directly, or through an {@link #update(UnaryOperator) update} Action) takes effect on
 * the next runtime cycle. Motors using the exact same profile instance share settings and form one
 * coupling group: their requested winding torques receive a common reduction, and large starts are
 * admitted together. Use {@link #copy()} for independent groups with common initial settings.</p>
 *
 * <ul>
 *   <li>{@link #sustained(Current)} – continuous winding (stator) current cap. Zero or unset means no cap.</li>
 *   <li>{@link #burst(Current, Time)} – higher cap allowed for a limited time budget above the
 *       sustained cap; the budget refills at half speed while the motor draws at or below sustained.</li>
 *   <li>{@link #rampLimit(Current, Time)} – maximum increase of the cap per time. Without it, a cap recovering
 *       from a sag or budget cut still rises gradually; see {@link PowerPlanner}.</li>
 *   <li>{@link #share(double)} – relative weight when the battery budget is short. The lowest share
 *       (relative to demand) is shed first; see {@link PowerBudget}.</li>
 *   <li>{@link #onSag(Sag)} – extra reduction while the battery sags, proportional to how far it sags.</li>
 * </ul>
 */
public final class PowerProfile {
    private double sustainedAmps;
    private double burstAmps;
    private double burstSeconds;
    private double rampStepAmps;
    private double rampIntervalSeconds;
    private double rampAmpsPerSecond;
    private double share = 1.0;
    private Sag sag;
    private String startupGroup;
    private PowerProfile declared;

    private PowerProfile() {
    }

    /** Creates an empty profile: no caps, share 1.0, no sag response. */
    public static PowerProfile of() {
        return new PowerProfile();
    }

    /**
     * Sets the sustained current cap.
     *
     * @param current cap, non-negative; zero removes the cap
     * @return this profile
     */
    public PowerProfile sustained(Current current) {
        sustainedAmps = nonNegativeAmps(current, "Sustained current");
        return this;
    }

    /**
     * Allows a burst current above the sustained cap for a limited time.
     *
     * @param current burst cap, non-negative
     * @param duration time budget for bursting, non-negative
     * @return this profile
     */
    public PowerProfile burst(Current current, Time duration) {
        burstAmps = nonNegativeAmps(current, "Burst current");
        burstSeconds = nonNegativeSeconds(duration, "Burst duration");
        return this;
    }

    /**
     * Limits how fast the cap may increase: at most {@code step} of additional current per
     * {@code interval}. Decreases are not limited by this rate; sag and budget cuts follow {@link PowerPlanner#SOFT_CUT_AMPS_PER_SECOND}.
     *
     * @param step maximum increase per interval, positive
     * @param interval interval the step applies over, positive
     * @return this profile
     */
    public PowerProfile rampLimit(Current step, Time interval) {
        double stepAmps = nonNegativeAmps(step, "Ramp step");
        double seconds = nonNegativeSeconds(interval, "Ramp interval");
        if (stepAmps <= 0.0 || seconds <= 0.0) {
            throw new IllegalArgumentException("Ramp step and interval must be positive.");
        }
        rampStepAmps = stepAmps;
        rampIntervalSeconds = seconds;
        rampAmpsPerSecond = stepAmps / seconds;
        return this;
    }

    /**
     * Sets this motor's weight when the battery budget is short.
     *
     * @param weight positive finite weight
     * @return this profile
     */
    public PowerProfile share(double weight) {
        if (!Double.isFinite(weight) || weight <= 0.0) {
            throw new IllegalArgumentException("Power share must be a positive finite weight.");
        }
        share = weight;
        return this;
    }

    /**
     * Sets the bus-sag response.
     *
     * @param sag sag response, or {@code null} to remove it
     * @return this profile
     */
    public PowerProfile onSag(Sag sag) {
        this.sag = sag;
        return this;
    }

    /**
     * Admits this profile's large starts together with other profiles using the same key, without
     * coupling their torque reductions. Motors sharing this profile already start together.
     *
     * <p>A {@link #copy() copy} does not inherit this relationship. Assign it explicitly when the new
     * independent profile should also join the startup group.</p>
     *
     * @param group shared startup key, or {@code null} to remove it
     * @return this profile
     */
    public PowerProfile startTogether(String group) {
        if (group != null && group.isBlank()) {
            throw new IllegalArgumentException("Startup group must not be blank.");
        }
        startupGroup = group;
        return this;
    }

    /** Returns the optional shared startup key; profile identity remains the torque coupling key. */
    public String startupGroup() { return startupGroup; }

    /** Returns the sustained cap, or zero when uncapped. */
    public Current sustainedCurrent() { return Units.Amps.of(sustainedAmps); }

    /** Returns the burst cap, or zero when no burst is declared. */
    public Current burstCurrent() { return Units.Amps.of(burstAmps); }

    /** Returns the burst time budget, or zero when no burst is declared. */
    public Time burstDuration() { return Units.Seconds.of(burstSeconds); }

    /** Returns the maximum cap increase per {@link #rampInterval()}, or zero when not ramp limited. */
    public Current rampStep() { return Units.Amps.of(rampStepAmps); }

    /** Returns the interval the {@link #rampStep()} applies over, or zero when not ramp limited. */
    public Time rampInterval() { return Units.Seconds.of(rampIntervalSeconds); }

    /** Per-cycle primitive: sustained cap in amps. Robot code should use {@link #sustainedCurrent()}. */
    double sustainedAmps() { return sustainedAmps; }

    /** Per-cycle primitive: burst cap in amps. Robot code should use {@link #burstCurrent()}. */
    double burstAmps() { return burstAmps; }

    /** Per-cycle primitive: burst budget in seconds. Robot code should use {@link #burstDuration()}. */
    double burstSeconds() { return burstSeconds; }

    /** Per-cycle primitive: ramp rate in amps per second. */
    double rampAmpsPerSecond() { return rampAmpsPerSecond; }

    /** Returns the relative share weight. */
    public double share() { return share; }

    /** Returns the sag response, or {@code null}. */
    public Sag sag() { return sag; }

    /**
     * Returns an Action that applies {@code change} to this profile when it runs. The change is
     * visible from the next runtime cycle.
     *
     * @param change mutation applied to this profile (typically returns {@code this})
     * @param <T> Action type
     * @return Action
     */
    public <T extends DeviceAction> T update(UnaryOperator<PowerProfile> change) {
        Objects.requireNonNull(change, "change");
        return PowerActions.doOnce(context -> {
            if (declared == null) {
                declared = snapshot();
            }
            change.apply(this);
        });
    }

    /**
     * Returns an Action that restores the declared values captured when this profile was first
     * attached to a motor or first updated.
     *
     * @param <T> Action type
     * @return Action
     */
    public <T extends DeviceAction> T restore() {
        return PowerActions.doOnce(context -> {
            if (declared != null) {
                copyFrom(declared);
            }
        });
    }

    /**
     * Captures the declared baseline used by {@link #restore()} on the first attachment. Attaching
     * another motor to an existing shared profile never overwrites that baseline.
     */
    public void markDeclared() {
        if (declared == null) {
            declared = snapshot();
        }
    }

    /**
     * Returns a detached copy of the current electrical settings. It has its own coupling identity,
     * restore baseline and runtime history. Explicit cross-profile startup membership is not copied.
     * Reusing the returned copy on several motors couples those motors to each other.
     */
    public PowerProfile copy() {
        PowerProfile copy = snapshot();
        copy.startupGroup = null;
        return copy;
    }

    private PowerProfile snapshot() {
        PowerProfile copy = new PowerProfile();
        copy.copyFrom(this);
        return copy;
    }

    private void copyFrom(PowerProfile other) {
        sustainedAmps = other.sustainedAmps;
        burstAmps = other.burstAmps;
        burstSeconds = other.burstSeconds;
        rampStepAmps = other.rampStepAmps;
        rampIntervalSeconds = other.rampIntervalSeconds;
        rampAmpsPerSecond = other.rampAmpsPerSecond;
        share = other.share;
        sag = other.sag;
        startupGroup = other.startupGroup;
    }

    private static double nonNegativeAmps(Current current, String name) {
        Objects.requireNonNull(current, name);
        double amps = current.in(Units.Amps);
        if (!Double.isFinite(amps) || amps < 0.0) {
            throw new IllegalArgumentException(name + " must be a non-negative finite current.");
        }
        return amps;
    }

    private static double nonNegativeSeconds(Time time, String name) {
        Objects.requireNonNull(time, name);
        double seconds = time.in(Units.Seconds);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            throw new IllegalArgumentException(name + " must be a non-negative finite time.");
        }
        return seconds;
    }

    @Override
    public String toString() {
        return "PowerProfile[sustainedAmps=" + sustainedAmps + ", burstAmps=" + burstAmps
                + ", burstSeconds=" + burstSeconds + ", rampAmpsPerSecond=" + rampAmpsPerSecond
                + ", share=" + share + ", sag=" + sag + "]";
    }

    /** Returns an internal, primitive view of this value. Not for robot code. */
    public Raw raw() {
        return new Raw(this);
    }

    /**
     * Internal, not for robot code: primitive profile values in the units the names state. Robot code uses the
     * typed accessors (sustainedCurrent(), burstCurrent(), burstDuration(), rampStep(), ...).
     */
    public static final class Raw {
        private final PowerProfile owner;

        private Raw(PowerProfile owner) {
            this.owner = owner;
        }

        /** Internal: see PowerProfile#sustainedAmps. */
        public double sustainedAmps() {
            return owner.sustainedAmps();
        }

        /** Internal: see PowerProfile#burstAmps. */
        public double burstAmps() {
            return owner.burstAmps();
        }

        /** Internal: see PowerProfile#burstSeconds. */
        public double burstSeconds() {
            return owner.burstSeconds();
        }

        /** Internal: see PowerProfile#rampAmpsPerSecond. */
        public double rampAmpsPerSecond() {
            return owner.rampAmpsPerSecond();
        }
    }
}
