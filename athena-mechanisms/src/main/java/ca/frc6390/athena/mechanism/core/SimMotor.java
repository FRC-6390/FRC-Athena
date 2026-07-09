package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;

final class SimMotor implements MotorHandle {
    enum CommandKind {
        NEUTRAL,
        PERCENT,
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    private final MotorHandle delegate;
    private CommandKind commandKind = CommandKind.NEUTRAL;
    private double commandValue;

    SimMotor(MotorHandle delegate) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
    }

    CommandKind commandKind() {
        return commandKind;
    }

    double commandValue() {
        return commandValue;
    }

    @Override
    public MotorDevice device() {
        return delegate.device();
    }

    @Override
    public void stop() {
        commandKind = CommandKind.NEUTRAL;
        commandValue = 0.0;
        delegate.stop();
    }

    @Override
    public void setPercentOutput(double percent) {
        commandKind = CommandKind.PERCENT;
        commandValue = percent;
        delegate.setPercentOutput(percent);
    }

    @Override
    public void setVoltage(double volts) {
        commandKind = CommandKind.VOLTAGE;
        commandValue = volts;
        delegate.setVoltage(volts);
    }

    @Override
    public void setPositionTargetRotations(double rotations) {
        commandKind = CommandKind.POSITION;
        commandValue = rotations;
        delegate.setPositionTargetRotations(rotations);
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        commandKind = CommandKind.VELOCITY;
        commandValue = rotationsPerSecond;
        delegate.setVelocityTargetRotationsPerSecond(rotationsPerSecond);
    }
}
