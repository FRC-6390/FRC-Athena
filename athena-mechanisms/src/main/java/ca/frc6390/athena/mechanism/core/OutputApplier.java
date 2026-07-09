package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.backend.MotorHandle;
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
        for (MotorDevice motor : motors(output.request())) {
            apply(context.motor(motor), output.output());
        }
    }

    private static List<MotorDevice> motors(OutputRequest request) {
        if (request.control() != null) {
            return request.control().motors();
        }
        if (request.motor() != null) {
            return List.of(request.motor());
        }
        return List.of();
    }

    private static void apply(MotorHandle motor, Output output) {
        if (output instanceof Output.Percent percent) {
            motor.setPercentOutput(percent.percent());
        } else if (output instanceof Output.Voltage voltage) {
            motor.setVoltage(voltage.volts());
        } else if (output instanceof Output.Position position) {
            motor.setPositionTargetRotations(position.position());
        } else if (output instanceof Output.Velocity velocity) {
            motor.setVelocityTargetRotationsPerSecond(velocity.velocity());
        } else if (output instanceof Output.Neutral || output instanceof Output.Fault) {
            motor.stop();
        }
    }
}
