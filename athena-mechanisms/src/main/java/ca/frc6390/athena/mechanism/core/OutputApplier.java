package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Applies resolved mechanism outputs to runtime hardware handles.
 */
public final class OutputApplier {
    private final ActionContext context;
    private final Map<ControlBinding, ControlRuntimeState> controlRuntimes = new HashMap<>();

    private OutputApplier(ActionContext context) {
        this.context = Objects.requireNonNull(context, "context");
    }

    public static OutputApplier using(ActionContext context) {
        return new OutputApplier(context);
    }

    public void applyAll(List<ResolvedOutput> outputs) {
        applyAll(outputs, MechanismContext.empty());
    }

    public void applyAll(List<ResolvedOutput> outputs, MechanismContext context) {
        Objects.requireNonNull(outputs, "outputs");
        MechanismContext safeContext = context == null ? MechanismContext.empty() : context;
        for (ResolvedOutput output : outputs) {
            apply(output, safeContext);
        }
    }

    public void apply(ResolvedOutput output) {
        apply(output, MechanismContext.empty());
    }

    public void apply(ResolvedOutput output, MechanismContext mechanismContext) {
        Objects.requireNonNull(output, "output");
        Output applied = resolveControlOutput(
                output,
                mechanismContext == null ? MechanismContext.empty() : mechanismContext);
        for (MotorDevice motor : motors(output.request())) {
            apply(context.motor(motor), applied);
        }
    }

    private Output resolveControlOutput(ResolvedOutput output, MechanismContext mechanismContext) {
        OutputRequest request = output.request();
        ControlBinding control = request.control();
        if (control == null || control.loops().isEmpty()) {
            return output.output();
        }
        ControlLoopContext loopContext = new AppliedControlLoopContext(control, output.output(), mechanismContext);
        ControlRuntimeState state = controlRuntimes(control);
        OutputKey key = OutputKey.of(output.output());
        if (!key.equals(state.lastRequest)) {
            state.runtimes.forEach(runtime -> runtime.reset(loopContext));
            state.lastRequest = key;
        }
        Output applied = null;
        for (ControlLoopRuntime runtime : state.runtimes) {
            ControlOutput controlOutput = runtime.calculate(loopContext);
            if (controlOutput != null) {
                applied = combine(applied, controlOutput.output());
            }
        }
        return applied == null ? output.output() : applied;
    }

    private ControlRuntimeState controlRuntimes(ControlBinding control) {
        return controlRuntimes.computeIfAbsent(control, binding -> new ControlRuntimeState(binding.loops().stream()
                .map(loop -> loop.bind(new ControlLoopBinding(binding, context)))
                .toList()));
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

    private static Output combine(Output current, Output next) {
        if (next == null) {
            return current;
        }
        if (current == null || current instanceof Output.Neutral || current instanceof Output.Fault
                || next instanceof Output.Neutral || next instanceof Output.Fault) {
            return next;
        }
        if (current instanceof Output.Voltage || next instanceof Output.Voltage) {
            return Outputs.voltage(volts(current) + volts(next));
        }
        if (current instanceof Output.Percent currentPercent && next instanceof Output.Percent nextPercent) {
            return Outputs.percent(currentPercent.percent() + nextPercent.percent());
        }
        if (current instanceof Output.Position currentPosition && next instanceof Output.Position nextPosition) {
            return Outputs.position(currentPosition.position() + nextPosition.position());
        }
        if (current instanceof Output.Velocity currentVelocity && next instanceof Output.Velocity nextVelocity) {
            return Outputs.velocity(currentVelocity.velocity() + nextVelocity.velocity());
        }
        return next;
    }

    private static double volts(Output output) {
        if (output instanceof Output.Voltage voltage) {
            return voltage.volts();
        }
        if (output instanceof Output.Percent percent) {
            return percent.percent() * 12.0;
        }
        return 0.0;
    }

    private final class AppliedControlLoopContext implements ControlLoopContext {
        private final ControlBinding control;
        private final Output request;
        private final MechanismContext mechanismContext;

        private AppliedControlLoopContext(ControlBinding control, Output request, MechanismContext mechanismContext) {
            this.control = control;
            this.request = request;
            this.mechanismContext = mechanismContext;
        }

        @Override
        public Output request() {
            return request;
        }

        @Override
        public double position() {
            return firstFeedbackPosition(control);
        }

        @Override
        public double velocity() {
            return firstFeedbackVelocity(control);
        }

        @Override
        public double dtSeconds() {
            return mechanismContext.dtSeconds();
        }

        @Override
        public ActionContext actionContext() {
            return context;
        }
    }

    private double firstFeedbackPosition(ControlBinding control) {
        if (!control.feedback().isEmpty()) {
            EncoderHandle encoder = context.encoder(control.feedback().get(0));
            return readOrZero(encoder::positionRotations);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrZero(motor::integratedPositionRotations);
        }
        return 0.0;
    }

    private double firstFeedbackVelocity(ControlBinding control) {
        if (!control.feedback().isEmpty()) {
            EncoderHandle encoder = context.encoder(control.feedback().get(0));
            return readOrZero(encoder::velocityRotationsPerSecond);
        }
        if (control.output() != null) {
            MotorHandle motor = context.motor(control.output());
            return readOrZero(motor::integratedVelocityRotationsPerSecond);
        }
        return 0.0;
    }

    private static double readOrZero(DoubleSupplier read) {
        try {
            return read.getAsDouble();
        } catch (UnsupportedOperationException ignored) {
            return 0.0;
        }
    }

    private interface DoubleSupplier {
        double getAsDouble();
    }

    private static final class ControlRuntimeState {
        private final List<ControlLoopRuntime> runtimes;
        private OutputKey lastRequest;

        private ControlRuntimeState(List<ControlLoopRuntime> runtimes) {
            this.runtimes = List.copyOf(runtimes);
        }
    }

    private record OutputKey(Class<?> type, double value) {
        private static OutputKey of(Output output) {
            return new OutputKey(output.getClass(), value(output));
        }

        private static double value(Output output) {
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
    }
}
