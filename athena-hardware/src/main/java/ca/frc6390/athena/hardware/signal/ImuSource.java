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
 * {@link #linearAccelerationX()}, ...). Each typed accessor is a default method over a primitive reading in
 * degrees, degrees per second or G. Athena's own per-cycle code reads those primitives through
 * {@link #raw()}, so the control path does not allocate measures.</p>
 *
 * <p>The {@code read*} and {@code writeYawDegrees} methods are the implementer SPI: a vendor or simulated source
 * overrides them. They are not the team API; use the typed accessors or {@link #raw()} instead.</p>
 *
 * <p>Control signals ({@link #headingSignal()}, {@link #yawRateSignal()}, ...) are base-unit feedback
 * signals for the control framework: radians, radians per second and G, documented per method.</p>
 */
public interface ImuSource {
    // Implementer SPI: primitive readings a source provides. Robot code does not call these; see raw().

    /** Yaw in degrees. Implementer SPI; robot code uses {@link #yaw()}. */
    double readYawDegrees();

    /** Pitch in degrees. Implementer SPI; robot code uses {@link #pitch()}. */
    double readPitchDegrees();

    /** Roll in degrees. Implementer SPI; robot code uses {@link #roll()}. */
    double readRollDegrees();

    /** Accumulated, unwrapped angle in degrees. Implementer SPI; robot code uses {@link #angle()}. */
    double readAngleDegrees();

    /** Yaw rate in degrees per second. Implementer SPI; robot code uses {@link #yawRate()}. */
    double readYawRateDegreesPerSecond();

    /** Pitch rate in degrees per second. Implementer SPI; robot code uses {@link #pitchRate()}. */
    default double readPitchRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU source does not provide pitch rate.");
    }

    /** Roll rate in degrees per second. Implementer SPI; robot code uses {@link #rollRate()}. */
    default double readRollRateDegreesPerSecond() {
        throw new UnsupportedOperationException("IMU source does not provide roll rate.");
    }

    /** X-axis linear acceleration in G. Implementer SPI; robot code uses {@link #linearAccelerationX()}. */
    double readLinearAccelerationXG();

    /** Y-axis linear acceleration in G. Implementer SPI; robot code uses {@link #linearAccelerationY()}. */
    double readLinearAccelerationYG();

    /** Z-axis linear acceleration in G. Implementer SPI; robot code uses {@link #linearAccelerationZ()}. */
    double readLinearAccelerationZG();

    /** Applies a yaw, in degrees, to the physical sensor. Implementer hook; robot code uses {@link #applyYaw}. */
    void writeYawDegrees(ActionContext context, double yawDegrees);

    // Typed readings.

    /** Returns the latest yaw. */
    default Angle yaw() {
        return Units.Degrees.of(readYawDegrees());
    }

    /** Returns the latest pitch. */
    default Angle pitch() {
        return Units.Degrees.of(readPitchDegrees());
    }

    /** Returns the latest roll. */
    default Angle roll() {
        return Units.Degrees.of(readRollDegrees());
    }

    /** Returns the accumulated, unwrapped angle. */
    default Angle angle() {
        return Units.Degrees.of(readAngleDegrees());
    }

    /** Returns the latest yaw rate. */
    default AngularVelocity yawRate() {
        return Units.DegreesPerSecond.of(readYawRateDegreesPerSecond());
    }

    /** Returns the latest pitch rate. */
    default AngularVelocity pitchRate() {
        return Units.DegreesPerSecond.of(readPitchRateDegreesPerSecond());
    }

    /** Returns the latest roll rate. */
    default AngularVelocity rollRate() {
        return Units.DegreesPerSecond.of(readRollRateDegreesPerSecond());
    }

    /** Returns the latest X-axis linear acceleration. */
    default LinearAcceleration linearAccelerationX() {
        return Units.Gs.of(readLinearAccelerationXG());
    }

    /** Returns the latest Y-axis linear acceleration. */
    default LinearAcceleration linearAccelerationY() {
        return Units.Gs.of(readLinearAccelerationYG());
    }

    /** Returns the latest Z-axis linear acceleration. */
    default LinearAcceleration linearAccelerationZ() {
        return Units.Gs.of(readLinearAccelerationZG());
    }

    // Control signals (base units, for the feedback framework).

    /** Returns wrapped robot heading as a position feedback signal in radians. */
    default PositionSignal headingSignal() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return normalizeRadians(Math.toRadians(source.readYawDegrees()));
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
            @Override public double position() { return Math.toRadians(source.readAngleDegrees()); }
            @Override public List<?> dependencies() { return source.dependencies(); }
        };
    }

    /** Returns pitch as a position feedback signal in radians. */
    default PositionSignal pitchSignal() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return Math.toRadians(source.readPitchDegrees());
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
                return Math.toRadians(source.readRollDegrees());
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
        return velocitySignal(source, source::readYawRateDegreesPerSecond);
    }

    /** Returns pitch rate in radians per second. */
    default VelocitySignal pitchRateSignal() {
        ImuSource source = this;
        return velocitySignal(source, source::readPitchRateDegreesPerSecond);
    }

    /** Returns roll rate in radians per second. */
    default VelocitySignal rollRateSignal() {
        ImuSource source = this;
        return velocitySignal(source, source::readRollRateDegreesPerSecond);
    }

    /** Returns robot X-axis linear acceleration as a scalar signal in G. */
    default ScalarSignal accelerationXSignal() {
        ImuSource source = this;
        return scalarSignal(source, source::readLinearAccelerationXG);
    }

    /** Returns robot Y-axis linear acceleration as a scalar signal in G. */
    default ScalarSignal accelerationYSignal() {
        ImuSource source = this;
        return scalarSignal(source, source::readLinearAccelerationYG);
    }

    /** Returns robot Z-axis linear acceleration as a scalar signal in G. */
    default ScalarSignal accelerationZSignal() {
        ImuSource source = this;
        return scalarSignal(source, source::readLinearAccelerationZG);
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
        writeYawDegrees(context, Objects.requireNonNull(yaw, "yaw").in(Units.Degrees));
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
    default double readLastUpdateSeconds() {
        return System.nanoTime() * 1.0e-9;
    }

    /** Returns whether this source is connected, calibrated, and no older than {@code maxAge}. */
    default boolean isFresh(Time maxAge) {
        double maxAgeSeconds = Objects.requireNonNull(maxAge, "maxAge").in(Units.Seconds);
        if (!Double.isFinite(maxAgeSeconds) || maxAgeSeconds < 0.0) {
            throw new IllegalArgumentException("Maximum IMU age must be finite and non-negative.");
        }
        double updated = readLastUpdateSeconds();
        return isConnected() && !isCalibrating() && Double.isFinite(updated)
                && System.nanoTime() * 1.0e-9 - updated <= maxAgeSeconds;
    }

    /** Returns an internal, primitive view of this source. Not for robot code: use the typed accessors. */
    default Raw raw() {
        return new Raw(this);
    }

    /**
     * Internal, not for robot code: primitive readings in degrees, degrees per second, G and seconds, for Athena's
     * per-cycle code. Robot code uses the typed accessors ({@link #yaw()}, {@link #yawRate()}, ...).
     */
    final class Raw {
        private final ImuSource source;

        private Raw(ImuSource source) {
            this.source = source;
        }

        /** Internal: yaw in degrees. */
        public double yawDegrees() {
            return source.readYawDegrees();
        }

        /** Internal: pitch in degrees. */
        public double pitchDegrees() {
            return source.readPitchDegrees();
        }

        /** Internal: roll in degrees. */
        public double rollDegrees() {
            return source.readRollDegrees();
        }

        /** Internal: accumulated, unwrapped angle in degrees. */
        public double angleDegrees() {
            return source.readAngleDegrees();
        }

        /** Internal: yaw rate in degrees per second. */
        public double yawRateDegreesPerSecond() {
            return source.readYawRateDegreesPerSecond();
        }

        /** Internal: pitch rate in degrees per second. */
        public double pitchRateDegreesPerSecond() {
            return source.readPitchRateDegreesPerSecond();
        }

        /** Internal: roll rate in degrees per second. */
        public double rollRateDegreesPerSecond() {
            return source.readRollRateDegreesPerSecond();
        }

        /** Internal: X-axis linear acceleration in G. */
        public double linearAccelerationXG() {
            return source.readLinearAccelerationXG();
        }

        /** Internal: Y-axis linear acceleration in G. */
        public double linearAccelerationYG() {
            return source.readLinearAccelerationYG();
        }

        /** Internal: Z-axis linear acceleration in G. */
        public double linearAccelerationZG() {
            return source.readLinearAccelerationZG();
        }

        /** Internal: monotonic timestamp of the latest update, in seconds. */
        public double lastUpdateSeconds() {
            return source.readLastUpdateSeconds();
        }

        /** Internal: applies a yaw, in degrees, to the physical sensor. */
        public void applyYawDegrees(ActionContext context, double yawDegrees) {
            source.writeYawDegrees(context, yawDegrees);
        }
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
