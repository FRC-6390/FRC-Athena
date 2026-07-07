package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionContext;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RuntimeMotor;
import java.util.List;
import java.util.Objects;

/**
 * Applies resolved mechanism outputs to runtime hardware handles.
 */
public final class OutputApplier {
    private final ActionContext context;

    private OutputApplier(ActionContext context) {
        this.context = Objects.requireNonNull(context, "context");
    }

    public static OutputApplier using(ActionContext context) {
        return new OutputApplier(context);
    }

    public void applyAll(List<ResolvedOutput> outputs) {
        Objects.requireNonNull(outputs, "outputs");
        for (ResolvedOutput output : outputs) {
            apply(output);
        }
    }

    public void apply(ResolvedOutput output) {
        Objects.requireNonNull(output, "output");
        for (MotorRef motor : motors(output.request())) {
            apply(context.motor(motor), output.output());
        }
    }

    private static List<MotorRef> motors(OutputRequest request) {
        if (request.control() != null) {
            return request.control().motors();
        }
        if (request.axis() != null) {
            return request.axis().motors();
        }
        if (request.motor() != null) {
            return List.of(request.motor());
        }
        return List.of();
    }

    private static void apply(RuntimeMotor motor, Output output) {
        if (output instanceof Output.Percent percent) {
            motor.percent(percent.percent());
        } else if (output instanceof Output.Voltage voltage) {
            motor.voltage(voltage.volts());
        } else if (output instanceof Output.Position position) {
            motor.position(position.position());
        } else if (output instanceof Output.Velocity velocity) {
            motor.velocity(velocity.velocity());
        } else if (output instanceof Output.Neutral || output instanceof Output.Fault) {
            motor.stop();
        }
    }
}
