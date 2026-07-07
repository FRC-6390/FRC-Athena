package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.MotorRef;
import java.util.Objects;

/**
 * Output request being evaluated by rules.
 */
public interface OutputRequest {
    ControlRef control();

    AxisRef axis();

    MotorRef motor();

    Output output();

    default boolean openLoop() {
        return output() instanceof Output.Percent || output() instanceof Output.Voltage;
    }

    default boolean closedLoop() {
        return output() instanceof Output.Position || output() instanceof Output.Velocity;
    }

    default boolean positive() {
        return value() > 0.0;
    }

    default boolean negative() {
        return value() < 0.0;
    }

    default double value() {
        Output output = output();
        if (output instanceof Output.Percent percent) {
            return percent.percent();
        }
        if (output instanceof Output.Voltage voltage) {
            return voltage.volts();
        }
        if (output instanceof Output.Position position) {
            return position.position();
        }
        if (output instanceof Output.Velocity velocity) {
            return velocity.velocity();
        }
        return 0.0;
    }

    static OutputRequest of(AxisRef axis, Output output) {
        return new Basic(null, axis, null, output);
    }

    static OutputRequest of(MotorRef motor, Output output) {
        return new Basic(null, null, motor, output);
    }

    static OutputRequest of(ControlRef control, Output output) {
        return new Basic(control, null, null, output);
    }

    record Basic(ControlRef control, AxisRef axis, MotorRef motor, Output output) implements OutputRequest {
        public Basic {
            Objects.requireNonNull(output, "output");
        }
    }
}
