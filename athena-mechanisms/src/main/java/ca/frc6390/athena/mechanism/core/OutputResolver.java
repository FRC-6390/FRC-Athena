package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Resolves requested mechanism Actions into hardware output requests.
 */
final class OutputResolver {
    private static final int STATIC_CACHE_LIMIT = 2048;
    private final RuntimeOverrides overrides;
    private final Map<Action, ResolvedOutput> staticOutputs = new IdentityHashMap<>();
    private OutputResolver(RuntimeOverrides overrides) {
        this.overrides = Objects.requireNonNull(overrides, "overrides");
    }

    static OutputResolver empty() {
        return new OutputResolver(new RuntimeOverrides());
    }

    RuntimeOverrides overrides() { return overrides; }

    public ResolvedOutput resolve(OutputRequest request) {
        Objects.requireNonNull(request, "request");
        return new ResolvedOutput(request, request.output());
    }

    public List<ResolvedOutput> resolve(Mechanism mechanism, Action action) {
        return resolve(mechanism, action, MechanismContext.empty());
    }

    public List<ResolvedOutput> resolve(Mechanism mechanism, Action action, MechanismContext context) {
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(action, "action");
        Objects.requireNonNull(context, "context");
        List<ResolvedOutput> outputs = new ArrayList<>();
        resolveState(action, context, outputs);
        return outputs;
    }

    void resolveInto(Mechanism mechanism, Action action, MechanismContext context, List<ResolvedOutput> outputs) {
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(action, "action");
        Objects.requireNonNull(context, "context");
        Objects.requireNonNull(outputs, "outputs");
        resolveState(action, context, outputs);
    }

    private void resolveState(Action action, MechanismContext context, List<ResolvedOutput> outputs) {
        if (action instanceof Actions.Conditional conditional) {
            resolveState(conditional.action(), context, outputs);
            return;
        }
        if (action instanceof Action.Conditional conditional) {
            resolveState(conditional.action(), context, outputs);
            return;
        }
        if (action instanceof Actions.Then then) {
            resolveState(then.action(), context, outputs);
            return;
        }
        if (action instanceof Action.Then then) {
            resolveState(then.action(), context, outputs);
            return;
        }
        if (action instanceof Actions.Timeout timeout) {
            if (!timeout.expired(context)) {
                resolveState(timeout.action(), context, outputs);
            }
            return;
        }
        if (action instanceof Actions.Choice choice) {
            resolveState(choice.choose(context), context, outputs);
            return;
        }
        if (action instanceof Actions.WhenBranch branch) {
            resolveState(branch.choose(context), context, outputs);
            return;
        }
        if (action instanceof Actions.Parallel parallel) {
            resolveStates(parallel.Actions(), context, outputs);
            return;
        }
        if (action instanceof MechanismRuntime.ScheduledOutputs scheduled) {
            resolveStates(scheduled.actions(), context, outputs);
            return;
        }
        if (action instanceof Actions.Race race) {
            resolveStates(race.Actions(), context, outputs);
            return;
        }
        if (action instanceof Actions.Deadline deadline) {
            resolveStates(deadline.Actions(), context, outputs);
            return;
        }
        if (action instanceof Actions.Sequence sequence) {
            if (context.timeInStateSeconds() >= sequence.timeoutSeconds()) {
                return;
            }
            if (!sequence.steps().isEmpty()) {
                resolveState(sequence.steps().get(0).action(), context, outputs);
                return;
            }
            if (sequence.next() != null) {
                resolveState(sequence.next(), context, outputs);
            }
            return;
        }
        if (action instanceof Actions.RuntimeAction || action instanceof Actions.DoOnce
                || action instanceof Actions.EncoderSetPosition
                || action instanceof Actions.WaitSeconds || action instanceof Actions.WaitUntil
                || action instanceof PathAction) {
            return;
        }
        if (action instanceof Actions.DynamicOutput dynamic) {
            resolveRequest(outputRequest(null, null, dynamic.output().apply(context)), outputs);
            return;
        }
        ControlBinding control = control(action);
        MotorDevice motor = motor(action);
        if (!available(control, motor)) return;
        Output output = output(action);
        if (output == null) return;
        if (staticOutput(action)) {
            ResolvedOutput resolved = staticOutputs.get(action);
            if (resolved == null) {
                if (staticOutputs.size() >= STATIC_CACHE_LIMIT) staticOutputs.clear();
                resolved = resolve(OutputRequest.Basic.of(control, motor, output));
                staticOutputs.put(action, resolved);
            }
            outputs.add(resolved);
            return;
        }
        resolveRequest(request(control, motor, output), outputs);
    }

    private void resolveStates(List<Action> actions, MechanismContext context, List<ResolvedOutput> outputs) {
        for (Action action : actions) {
            resolveState(action, context, outputs);
        }
    }

    private void resolveRequest(OutputRequest request, List<ResolvedOutput> outputs) {
        if (request != null) {
            outputs.add(resolve(request));
        }
    }

    private OutputRequest outputRequest(ControlBinding control, MotorDevice motor, Output output) {
        if (output == null || !available(control, motor)) return null;
        return request(control, motor, output);
    }

    private boolean available(ControlBinding control, MotorDevice motor) {
        if (control != null && (overrides.disabled(control)
                || (control.output() != null && overrides.disabled(control.output())))) {
            return false;
        }
        if (motor != null && overrides.disabled(motor)) {
            return false;
        }
        if (control != null && control.motors().stream().anyMatch(overrides::disabled)) return false;
        return true;
    }

    private static OutputRequest request(ControlBinding control, MotorDevice motor, Output output) {
        if (control != null) {
            return OutputRequest.of(control, output);
        }
        if (motor != null) {
            return OutputRequest.of(motor, output);
        }
        return null;
    }

    private static boolean staticOutput(Action action) {
        return action instanceof Actions.ControlNeutral
                || action instanceof Actions.MotorNeutral
                || action instanceof Actions.MotorPercent
                || action instanceof Actions.MotorVoltage
                || action instanceof Actions.ControlPercent
                || action instanceof Actions.ControlVoltage
                || action instanceof Actions.ControlPosition
                || action instanceof Actions.ControlVelocity;
    }

    private static ControlBinding control(Action action) {
        if (action instanceof Actions.ControlNeutral controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.ControlPercent controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.DynamicControlPercent controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.ControlVoltage controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.DynamicControlVoltage controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.ControlSysIdVoltage sysId) {
            return sysId.routine().control();
        }
        if (action instanceof Actions.ControlPosition controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.DynamicControlPosition controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.ControlVelocity controlState) {
            return controlState.control();
        }
        if (action instanceof Actions.DynamicControlVelocity controlState) {
            return controlState.control();
        }
        if (action instanceof InterpolatedControlAction interpolated) {
            return interpolated.control();
        }
        return null;
    }

    private static MotorDevice motor(Action action) {
        if (action instanceof Actions.MotorNeutral motorState) {
            return motorState.motor();
        }
        if (action instanceof Actions.MotorPercent motorState) {
            return motorState.motor();
        }
        if (action instanceof Actions.DynamicMotorPercent motorState) {
            return motorState.motor();
        }
        if (action instanceof Actions.MotorVoltage motorState) {
            return motorState.motor();
        }
        if (action instanceof Actions.DynamicMotorVoltage motorState) {
            return motorState.motor();
        }
        return null;
    }

    private static Output output(Action action) {
        // Freeze dynamic suppliers once per runtime cycle. Output application, arbitration, and
        // telemetry must all observe the same value without evaluating robot code again.
        if (action instanceof Actions.DynamicMotorPercent value) {
            return Outputs.percent(value.percent());
        }
        if (action instanceof Actions.DynamicMotorVoltage value) {
            return Outputs.voltage(value.volts());
        }
        if (action instanceof Actions.DynamicControlPercent value) {
            return Outputs.percent(value.percent());
        }
        if (action instanceof Actions.DynamicControlVoltage value) {
            return Outputs.voltage(value.volts());
        }
        if (action instanceof Actions.DynamicControlPosition value) {
            return Outputs.position(value.position());
        }
        if (action instanceof Actions.DynamicControlVelocity value) {
            return Outputs.velocity(value.velocity());
        }
        if (action instanceof InterpolatedControlAction interpolated) {
            double target = interpolated.target();
            return interpolated.control().mode() == ControlMode.VELOCITY
                    ? Outputs.velocity(target)
                    : Outputs.position(target);
        }
        return action instanceof Output output ? output : null;
    }
}
