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

    double linearAccelerationXG();

    double linearAccelerationYG();

    double linearAccelerationZG();

    /** Returns yaw as a position feedback signal in radians. */
    default PositionSignal yaw() {
        ImuSource source = this;
        return new PositionSignal() {
            @Override
            public double position() {
                return Math.toRadians(source.yawDegrees());
            }

            @Override
            public List<?> dependencies() {
                return source.dependencies();
            }
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

    default ImuSource relative() {
        return Imus.relative(this);
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

    void applyYaw(ActionContext context, double yawDegrees);
}
