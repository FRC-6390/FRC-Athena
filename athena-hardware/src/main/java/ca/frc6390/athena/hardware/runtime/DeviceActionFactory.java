package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.signal.ImuSource;
import edu.wpi.first.units.measure.Time;
import java.util.function.DoubleSupplier;

/**
 * Creates the Athena Actions that hardware declarations return. The Action types live in athena-mechanisms,
 * which depends on this module, so the implementation is supplied by athena-mechanisms through
 * {@link java.util.ServiceLoader} and looked up by {@link DeviceActionFactories}.
 */
public interface DeviceActionFactory {
    DeviceAction doOnce(ActionBinding binding);

    DeviceAction motorNeutral(MotorDevice motor);

    DeviceAction motorPercent(MotorDevice motor, double percent);

    DeviceAction motorPercent(MotorDevice motor, DoubleSupplier percent);

    DeviceAction motorVoltage(MotorDevice motor, double volts);

    DeviceAction motorVoltage(MotorDevice motor, DoubleSupplier volts);

    DeviceAction encoderSetPosition(EncoderDevice encoder, double position);

    DeviceAction imuSetYaw(ImuSource imu, double yawDegrees);

    DeviceAction imuSetYaw(ImuSource imu, DoubleSupplier yawDegrees);

    /** Runs {@code next} after {@code action} completes. */
    DeviceAction then(DeviceAction action, DeviceAction next);

    /** Completes {@code action} after {@code duration}. */
    DeviceAction timeout(DeviceAction action, Time duration);

    DeviceAction untilStalled(DeviceAction action, ca.frc6390.athena.hardware.signal.MotorStallSignal signal);
}
