package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.List;
import java.util.Objects;

/**
 * Output request emitted by a Action before it is applied to hardware.
 */
public interface OutputRequest {
    ControlBinding control();

    MotorDevice motor();

    Output output();

    default List<MotorDevice> motors() {
        if (control() != null) {
            return control().motors();
        }
        return motor() == null ? List.of() : List.of(motor());
    }

    default boolean openLoop() {
        return output() instanceof Output.Percent || output() instanceof Output.Voltage;
    }

    default boolean closedLoop() {
        return output() instanceof Output.Position || output() instanceof Output.Velocity;
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

    static OutputRequest of(MotorDevice motor, Output output) {
        return new Basic(null, motor, output);
    }

    static OutputRequest of(ControlBinding control, Output output) {
        return new Basic(control, null, output);
    }

    record Basic(ControlBinding control, MotorDevice motor, Output output) implements OutputRequest {
        public Basic {
            Objects.requireNonNull(output, "output");
        }

        static Basic of(ControlBinding control, MotorDevice motor, Output output) {
            return new Basic(control, motor, output);
        }
    }
}
