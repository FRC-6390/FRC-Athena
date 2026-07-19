package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Runtime orientation and inertial readings from a physical or derived source.
 */
public interface ImuSource {
    double yawDegrees();

    double pitchDegrees();

    double rollDegrees();

    double angleDegrees();

    double yawRateDegreesPerSecond();

    default double pitchRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU source does not provide pitch rate.");
    }

    default double rollRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU source does not provide roll rate.");
    }

    double linearAccelerationXG();

    double linearAccelerationYG();

    double linearAccelerationZG();

    /** Returns wrapped robot heading as a position feedback signal in radians. */
    default PositionSignal heading() {
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

    /** Returns wrapped yaw in radians. Alias for {@link #heading()}. */
    default PositionSignal yaw() {
        return heading();
    }

    /** Returns accumulated, unwrapped heading as a position feedback signal in radians. */
    default PositionSignal continuousHeading() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override public double position() { return Math.toRadians(source.angleDegrees()); }
            @Override public List<?> dependencies() { return source.dependencies(); }
        };
    }

    /** Returns pitch as a position feedback signal in radians. */
    default PositionSignal pitch() {
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
    default PositionSignal roll() {
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
    default VelocitySignal yawRate() {
        ImuSource source = this;
        return new VelocitySignal() {
            @Override
            public double velocity() {
                return Math.toRadians(source.yawRateDegreesPerSecond());
            }

            @Override
            public List<?> dependencies() {
                return source.dependencies();
            }
        };
    }

    /** Returns pitch rate in radians per second. */
    default VelocitySignal pitchRate() {
        ImuSource source = this;
        return velocitySignal(source, source::pitchRateDegreesPerSecond);
    }

    /** Returns roll rate in radians per second. */
    default VelocitySignal rollRate() {
        ImuSource source = this;
        return velocitySignal(source, source::rollRateDegreesPerSecond);
    }

    /** Returns robot X-axis linear acceleration in G. */
    default ScalarSignal accelerationX() {
        ImuSource source = this;
        return scalarSignal(source, source::linearAccelerationXG);
    }

    /** Returns robot Y-axis linear acceleration in G. */
    default ScalarSignal accelerationY() {
        ImuSource source = this;
        return scalarSignal(source, source::linearAccelerationYG);
    }

    /** Returns robot Z-axis linear acceleration in G. */
    default ScalarSignal accelerationZ() {
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

    default <T extends DeviceAction> T setYaw(double yawDegrees) {
        return Imus.setYawAction(this, yawDegrees);
    }

    /** Creates an action that reads the desired yaw when the action executes. */
    default <T extends DeviceAction> T setYaw(DoubleSupplier yawDegrees) {
        return Imus.setYawAction(this, yawDegrees);
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

    /** Returns whether this source is connected, calibrated, and no older than {@code maxAgeSeconds}. */
    default boolean isFresh(double maxAgeSeconds) {
        if (!Double.isFinite(maxAgeSeconds) || maxAgeSeconds < 0.0) {
            throw new IllegalArgumentException("Maximum IMU age must be finite and non-negative.");
        }
        double updated = lastUpdateSeconds();
        return isConnected() && !isCalibrating() && Double.isFinite(updated)
                && System.nanoTime() * 1.0e-9 - updated <= maxAgeSeconds;
    }

    void applyYaw(ActionContext context, double yawDegrees);

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
