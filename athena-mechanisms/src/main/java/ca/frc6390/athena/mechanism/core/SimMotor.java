package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.RuntimeMotor;
import java.util.Objects;

final class SimMotor implements RuntimeMotor {
    enum CommandKind {
        NEUTRAL,
        PERCENT,
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    private final RuntimeMotor delegate;
    private CommandKind commandKind = CommandKind.NEUTRAL;
    private double commandValue;

    SimMotor(RuntimeMotor delegate) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
    }

    CommandKind commandKind() {
        return commandKind;
    }

    double commandValue() {
        return commandValue;
    }

    @Override
    public void stop() {
        commandKind = CommandKind.NEUTRAL;
        commandValue = 0.0;
        delegate.stop();
    }

    @Override
    public void percent(double output) {
        commandKind = CommandKind.PERCENT;
        commandValue = output;
        delegate.percent(output);
    }

    @Override
    public void voltage(double volts) {
        commandKind = CommandKind.VOLTAGE;
        commandValue = volts;
        delegate.voltage(volts);
    }

    @Override
    public void position(double position) {
        commandKind = CommandKind.POSITION;
        commandValue = position;
        delegate.position(position);
    }

    @Override
    public void velocity(double velocity) {
        commandKind = CommandKind.VELOCITY;
        commandValue = velocity;
        delegate.velocity(velocity);
    }

    @Override
    public void brake() {
        delegate.brake();
    }

    @Override
    public void coast() {
        delegate.coast();
    }
}
