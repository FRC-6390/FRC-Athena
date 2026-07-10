package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.util.List;

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

    default ImuSource relative() {
        return Imus.relative(this);
    }

    default <T extends DeviceAction> T setYaw(double yawDegrees) {
        return Imus.setYawAction(this, yawDegrees);
    }

    default List<?> dependencies() {
        return List.of();
    }

    void applyYaw(ActionContext context, double yawDegrees);
}
