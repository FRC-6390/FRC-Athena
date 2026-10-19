package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.runtime.DeviceActionFactory;
import ca.frc6390.athena.hardware.signal.ImuSource;
import java.util.function.DoubleSupplier;

/**
 * Creates mechanism Actions for hardware declarations. athena-hardware cannot reference Action, so it
 * finds this class through {@link java.util.ServiceLoader} (see {@code META-INF/services}). Applications
 * do not use it directly.
 */
public final class MechanismDeviceActions implements DeviceActionFactory {
    public MechanismDeviceActions() {
    }

    @Override
    public DeviceAction doOnce(ActionBinding binding) {
        return Actions.doOnce(binding);
    }

    @Override
    public DeviceAction motorNeutral(MotorDevice motor) {
        return Actions.neutral(motor);
    }

    @Override
    public DeviceAction motorPercent(MotorDevice motor, double percent) {
        return Actions.percent(motor, percent);
    }

    @Override
    public DeviceAction motorPercent(MotorDevice motor, DoubleSupplier percent) {
        return Actions.percent(motor, percent);
    }

    @Override
    public DeviceAction motorVoltage(MotorDevice motor, double volts) {
        return Actions.voltage(motor, volts);
    }

    @Override
    public DeviceAction motorVoltage(MotorDevice motor, DoubleSupplier volts) {
        return Actions.voltage(motor, volts);
    }

    @Override
    public DeviceAction encoderSetPosition(EncoderDevice encoder, double position) {
        return Actions.setPosition(encoder, position);
    }

    @Override
    public DeviceAction imuSetYaw(ImuSource imu, double yawDegrees) {
        return Actions.setYaw(imu, yawDegrees);
    }

    @Override
    public DeviceAction imuSetYaw(ImuSource imu, DoubleSupplier yawDegrees) {
        return Actions.setYaw(imu, yawDegrees);
    }

    @Override
    public DeviceAction then(DeviceAction action, DeviceAction next) {
        return Actions.then(action, next);
    }

    @Override
    public DeviceAction timeout(DeviceAction action, double seconds) {
        return Actions.timeout(action, seconds);
    }

    @Override
    public DeviceAction untilStalled(DeviceAction action, ca.frc6390.athena.hardware.signal.MotorStallSignal signal) {
        return Actions.untilStalled(action, signal);
    }
}
