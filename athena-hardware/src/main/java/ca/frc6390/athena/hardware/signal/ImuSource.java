package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Runtime orientation and inertial readings from a physical or derived source.
 *
 * <p>Robot code reads the typed accessors ({@link #yaw()}, {@link #yawRate()},
 * {@link #linearAccelerationX()}, ...). Each typed accessor is a default method over a primitive
 * per-cycle reading in degrees, degrees per second or G. Implementers provide the primitives; Athena's
 * derived sources (mounting, inversion, relative, transform) use them so the control path does not
 * allocate measures.</p>
 *
 * <p>Control signals ({@link #headingSignal()}, {@link #yawRateSignal()}, ...) are base-unit feedback
 * signals for the control framework: radians, radians per second and G, documented per method.</p>
 */
public interface ImuSource {
    // Per-cycle primitives. Implementers provide these; robot code should use the typed accessors.

    /** Yaw in degrees. Per-cycle primitive; teams use {@link #yaw()}. */
    double yawDegrees();

    /** Pitch in degrees. Per-cycle primitive; teams use {@link #pitch()}. */
    double pitchDegrees();

    /** Roll in degrees. Per-cycle primitive; teams use {@link #roll()}. */
    double rollDegrees();

    /** Accumulated, unwrapped angle in degrees. Per-cycle primitive; teams use {@link #angle()}. */
    double angleDegrees();

    /** Yaw rate in degrees per second. Per-cycle primitive; teams use {@link #yawRate()}. */
    double yawRateDegreesPerSecond();

    /** Pitch rate in degrees per second. Per-cycle primitive; teams use {@link #pitchRate()}. */
    default double pitchRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU source does not provide pitch rate.");
    }

    /** Roll rate in degrees per second. Per-cycle primitive; teams use {@link #rollRate()}. */
    default double rollRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU source does not provide roll rate.");
    }

    /** X-axis linear acceleration in G. Per-cycle primitive; teams use {@link #linearAccelerationX()}. */
    double linearAccelerationXG();

    /** Y-axis linear acceleration in G. Per-cycle primitive; teams use {@link #linearAccelerationY()}. */
    double linearAccelerationYG();

    /** Z-axis linear acceleration in G. Per-cycle primitive; teams use {@link #linearAccelerationZ()}. */
    double linearAccelerationZG();

    /** Applies a yaw, in degrees, to the physical sensor. Per-cycle primitive; teams use {@link #applyYaw}. */
    void applyYawDegrees(ActionContext context, double yawDegrees);

    // Typed readings.

    /** Returns the latest yaw. */
    default Angle yaw() {
        return Units.Degrees.of(yawDegrees());
    }

    /** Returns the latest pitch. */
    default Angle pitch() {
        return Units.Degrees.of(pitchDegrees());
    }

    /** Returns the latest roll. */
    default Angle roll() {
        return Units.Degrees.of(rollDegrees());
    }

    /** Returns the accumulated, unwrapped angle. */
    default Angle angle() {
        return Units.Degrees.of(angleDegrees());
    }

    /** Returns the latest yaw rate. */
    default AngularVelocity yawRate() {
        return Units.DegreesPerSecond.of(yawRateDegreesPerSecond());
    }

    /** Returns the latest pitch rate. */
    default AngularVelocity pitchRate() {
        return Units.DegreesPerSecond.of(pitchRateDegreesPerSecond());
    }

    /** Returns the latest roll rate. */
    default AngularVelocity rollRate() {
        return Units.DegreesPerSecond.of(rollRateDegreesPerSecond());
    }

    /** Returns the latest X-axis linear acceleration. */
    default LinearAcceleration linearAccelerationX() {
        return Units.Gs.of(linearAccelerationXG());
    }

    /** Returns the latest Y-axis linear acceleration. */
    default LinearAcceleration linearAccelerationY() {
        return Units.Gs.of(linearAccelerationYG());
    }

    /** Returns the latest Z-axis linear acceleration. */
    default LinearAcceleration linearAccelerationZ() {
        return Units.Gs.of(linearAccelerationZG());
    }

    // Control signals (base units, for the feedback framework).

    /** Returns wrapped robot heading as a position feedback signal in radians. */
    default PositionSignal headingSignal() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return normalizeRadians(Math.toRadians(source.yawDegrees()));
            }

            @Override
            public List<?> dependencies() {
                return source.dependencies();
            }
        };
    }

    /** Returns accumulated, unwrapped heading as a position feedback signal in radians. */
    default PositionSignal continuousHeadingSignal() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override public double position() { return Math.toRadians(source.angleDegrees()); }
            @Override public List<?> dependencies() { return source.dependencies(); }
        };
    }

    /** Returns pitch as a position feedback signal in radians. */
    default PositionSignal pitchSignal() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return Math.toRadians(source.pitchDegrees());
            }

            @Override
            public List<?> dependencies() {
                return source.dependencies();
            }
        };
    }

    /** Returns roll as a position feedback signal in radians. */
    default PositionSignal rollSignal() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return Math.toRadians(source.rollDegrees());
            }

            @Override
            public List<?> dependencies() {
                return source.dependencies();
            }
        };
    }

    /** Returns yaw rate as a velocity feedback signal in radians per second. */
    default VelocitySignal yawRateSignal() {
        ImuSource source = this;
        return velocitySignal(source, source::yawRateDegreesPerSecond);
    }

    /** Returns pitch rate in radians per second. */
    default VelocitySignal pitchRateSignal() {
        ImuSource source = this;
        return velocitySignal(source, source::pitchRateDegreesPerSecond);
    }

    /** Returns roll rate in radians per second. */
    default VelocitySignal rollRateSignal() {
        ImuSource source = this;
        return velocitySignal(source, source::rollRateDegreesPerSecond);
    }

    /** Returns robot X-axis linear acceleration as a scalar signal in G. */
    default ScalarSignal accelerationXSignal() {
        ImuSource source = this;
        return scalarSignal(source, source::linearAccelerationXG);
    }

    /** Returns robot Y-axis linear acceleration as a scalar signal in G. */
    default ScalarSignal accelerationYSignal() {
        ImuSource source = this;
        return scalarSignal(source, source::linearAccelerationYG);
    }

    /** Returns robot Z-axis linear acceleration as a scalar signal in G. */
    default ScalarSignal accelerationZSignal() {
        ImuSource source = this;
        return scalarSignal(source, source::linearAccelerationZG);
    }

    default ImuSource relative() {
        return Imus.relative(this);
    }

    /**
     * Returns a view with every orientation, rate, and acceleration reading negated.
     * Setting yaw through the view uses the same inverted coordinate convention.
     *
     * @return inverted IMU source
     */
    default ImuSource inverted() {
        return inverted(true);
    }

    /**
     * Selects whether this source is viewed with all readings inverted.
     *
     * @param inverted whether to invert the source
     * @return this source when false, otherwise an inverted view
     */
    default ImuSource inverted(boolean inverted) {
        return inverted ? Imus.inverted(this) : this;
    }

    /** Applies a signed vendor-axis convention transform. */
    default ImuSource transform(ImuTransform transform) {
        return Imus.transform(this, transform);
    }

    /** Applies the sensor's physical mounting orientation relative to the robot. */
    default ImuSource mounting(ImuMount mount) {
        return Imus.mounting(this, mount);
    }

    /** Creates an action that sets this source's yaw. */
    default <T extends DeviceAction> T setYaw(Angle yaw) {
        return Imus.setYawAction(this, Objects.requireNonNull(yaw, "yaw"));
    }

    /** Creates an action that reads the desired yaw when the action executes. */
    default <T extends DeviceAction> T setYaw(Supplier<Angle> yaw) {
        return Imus.setYawAction(this, Objects.requireNonNull(yaw, "yaw"));
    }

    /** Applies a yaw to the physical sensor. */
    default void applyYaw(ActionContext context, Angle yaw) {
        applyYawDegrees(context, Objects.requireNonNull(yaw, "yaw").in(Units.Degrees));
    }

    default List<?> dependencies() {
        return List.of();
    }

    /** Returns whether this source is currently connected. */
    default boolean isConnected() {
        return true;
    }

    /** Returns whether this source is currently calibrating and should not be trusted yet. */
    default boolean isCalibrating() {
        return false;
    }

    /** Returns the monotonic timestamp of the latest successful update, in seconds. */
    default double lastUpdateSeconds() {
        return System.nanoTime() * 1.0e-9;
    }

    /** Returns whether this source is connected, calibrated, and no older than {@code maxAge}. */
    default boolean isFresh(Time maxAge) {
        double maxAgeSeconds = Objects.requireNonNull(maxAge, "maxAge").in(Units.Seconds);
        if (!Double.isFinite(maxAgeSeconds) || maxAgeSeconds < 0.0) {
            throw new IllegalArgumentException("Maximum IMU age must be finite and non-negative.");
        }
        double updated = lastUpdateSeconds();
        return isConnected() && !isCalibrating() && Double.isFinite(updated)
                && System.nanoTime() * 1.0e-9 - updated <= maxAgeSeconds;
    }

    private static VelocitySignal velocitySignal(ImuSource source, DoubleSupplier reading) {
        return new VelocitySignal() {
            @Override public double velocity() { return Math.toRadians(reading.getAsDouble()); }
            @Override public List<?> dependencies() { return source.dependencies(); }
        };
    }

    private static ScalarSignal scalarSignal(ImuSource source, DoubleSupplier reading) {
        return new ScalarSignal() {
            @Override public double value() { return reading.getAsDouble(); }
            @Override public List<?> dependencies() { return source.dependencies(); }
        };
    }

    private static double normalizeRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
