package ca.frc6390.athena.mechanism.core;

/**
 * Output produced by a control-loop runtime.
 */
public sealed interface ControlOutput
        permits ControlOutput.Percent, ControlOutput.Voltage, ControlOutput.Position, ControlOutput.Velocity,
        ControlOutput.Neutral {
    /**
     * Converts loop output to Athena output.
     *
     * @return output
     */
    Output output();

    static ControlOutput percent(double value) {
        return new Percent(value);
    }

    /**
     * Voltage output.
     *
     * @param value volts
     * @return output
     */
    static ControlOutput voltage(edu.wpi.first.units.measure.Voltage value) {
        return new Voltage(value.in(edu.wpi.first.units.Units.Volts));
    }

    /**
     * Primitive voltage output for per-cycle control loops, in volts. Team code uses the {@code Voltage} form.
     *
     * @param value volts
     * @return output
     */
    static ControlOutput voltage(double value) {
        return new Voltage(value);
    }

    static ControlOutput position(double value) {
        return new Position(value);
    }

    static ControlOutput velocity(double value) {
        return new Velocity(value);
    }

    static ControlOutput neutral() {
        return new Neutral();
    }

    record Percent(double value) implements ControlOutput {
        @Override
        public Output output() {
            return Outputs.percent(value);
        }
    }

    record Voltage(double value) implements ControlOutput {
        @Override
        public Output output() {
            return Outputs.voltage(value);
        }
    }

    record Position(double value) implements ControlOutput {
        @Override
        public Output output() {
            return Outputs.position(value);
        }
    }

    record Velocity(double value) implements ControlOutput {
        @Override
        public Output output() {
            return Outputs.velocity(value);
        }
    }

    record Neutral() implements ControlOutput {
        @Override
        public Output output() {
            return Outputs.neutral();
        }
    }
}
