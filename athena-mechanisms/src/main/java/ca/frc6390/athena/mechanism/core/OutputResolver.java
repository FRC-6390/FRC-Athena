package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Resolves requested mechanism Actions into hardware output requests.
 */
final class OutputResolver {
    private OutputResolver() {
    }

    static OutputResolver empty() {
        return new OutputResolver();
    }

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
        resolveRequest(outputRequest(control(action), motor(action), output(action)), outputs);
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

    private static OutputRequest outputRequest(ControlBinding control, MotorDevice motor, Output output) {
        if (output == null) {
            return null;
        }
        if (control != null && (control.isDisabled()
                || (control.output() != null && control.output().isDisabled()))) {
            return null;
        }
        if (motor != null && motor.isDisabled()) {
            return null;
        }
        if (control != null) {
            return OutputRequest.of(control, output);
        }
        if (motor != null) {
            return OutputRequest.of(motor, output);
        }
        return null;
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
        return action instanceof Output output ? output : null;
    }
}
