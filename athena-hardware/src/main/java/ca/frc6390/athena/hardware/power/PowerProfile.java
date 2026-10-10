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
 * the next runtime cycle. Every motor using a profile shares its settings.</p>
 *
 * <ul>
 *   <li>{@link #sustained(Current)} – continuous current cap. Zero or unset means no cap.</li>
 *   <li>{@link #burst(Current, Time)} – higher cap allowed for a limited time budget above the
 *       sustained cap; the budget refills at half speed while the motor draws at or below sustained.</li>
 *   <li>{@link #rampLimit(Current, Time)} – maximum increase of the cap per time (decreases are immediate).</li>
 *   <li>{@link #share(double)} – relative weight when the battery budget is short. The lowest share
 *       (relative to demand) is shed first; see {@link PowerBudget}.</li>
 *   <li>{@link #onSag(Sag)} – extra reduction while the battery sags.</li>
 * </ul>
 */
public final class PowerProfile {
    private double sustainedAmps;
    private double burstAmps;
    private double burstSeconds;
    private double rampAmpsPerSecond;
    private double share = 1.0;
    private Sag sag;
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
     * {@code interval}. Decreases are never ramp limited.
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

    /** Returns the sustained cap in amps, or zero when uncapped. */
    public double sustainedAmps() { return sustainedAmps; }

    /** Returns the burst cap in amps, or zero when no burst is declared. */
    public double burstAmps() { return burstAmps; }

    /** Returns the burst time budget in seconds. */
    public double burstSeconds() { return burstSeconds; }

    /** Returns the maximum cap increase in amps per second, or zero when not ramp limited. */
    public double rampAmpsPerSecond() { return rampAmpsPerSecond; }

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
                declared = copy();
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

    /** Captures the current values as the declared baseline used by {@link #restore()}. */
    public void markDeclared() {
        declared = copy();
    }

    /** Returns a detached copy of the current values. */
    public PowerProfile copy() {
        PowerProfile copy = new PowerProfile();
        copy.copyFrom(this);
        return copy;
    }

    private void copyFrom(PowerProfile other) {
        sustainedAmps = other.sustainedAmps;
        burstAmps = other.burstAmps;
        burstSeconds = other.burstSeconds;
        rampAmpsPerSecond = other.rampAmpsPerSecond;
        share = other.share;
        sag = other.sag;
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
}
