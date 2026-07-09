package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Resolves requested mechanism states into hardware output requests.
 */
public final class OutputResolver {
    private OutputResolver() {
    }

    public static OutputResolver empty() {
        return new OutputResolver();
    }

    public ResolvedOutput resolve(OutputRequest request) {
        Objects.requireNonNull(request, "request");
        return new ResolvedOutput(request, request.output());
    }

    public List<ResolvedOutput> resolve(Mechanism mechanism, State state) {
        return resolve(mechanism, state, MechanismContext.empty());
    }

    public List<ResolvedOutput> resolve(Mechanism mechanism, State state, MechanismContext context) {
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(context, "context");
        return resolveState(state, context);
    }

    private List<ResolvedOutput> resolveState(State state, MechanismContext context) {
        if (state instanceof States.ChildSet childSet) {
            List<ResolvedOutput> outputs = new ArrayList<>();
            for (States.ChildTarget target : childSet.targets()) {
                outputs.addAll(resolveState(target.state(), context));
            }
            return outputs;
        }
        if (state instanceof States.Clamped clamped) {
            return resolveState(clamped.state(), context);
        }
        if (state instanceof State.Clamped clamped) {
            return resolveState(clamped.state(), context);
        }
        if (state instanceof States.Conditional conditional) {
            return resolveState(conditional.state(), context);
        }
        if (state instanceof State.Conditional conditional) {
            return resolveState(conditional.state(), context);
        }
        if (state instanceof States.Then then) {
            return resolveState(then.state(), context);
        }
        if (state instanceof State.Then then) {
            return resolveState(then.state(), context);
        }
        if (state instanceof States.Timeout timeout) {
            return timeout.expired(context) ? List.of() : resolveState(timeout.state(), context);
        }
        if (state instanceof States.Choice choice) {
            return resolveState(choice.choose(context), context);
        }
        if (state instanceof States.WhenBranch branch) {
            return resolveState(branch.choose(context), context);
        }
        if (state instanceof States.Parallel parallel) {
            return resolveStates(parallel.states(), context);
        }
        if (state instanceof States.Race race) {
            return resolveStates(race.states(), context);
        }
        if (state instanceof States.Deadline deadline) {
            return resolveStates(deadline.states(), context);
        }
        if (state instanceof States.Sequence sequence) {
            if (context.timeInStateSeconds() >= sequence.timeoutSeconds()) {
                return List.of();
            }
            if (!sequence.steps().isEmpty()) {
                return resolveState(sequence.steps().get(0).state(), context);
            }
            return sequence.next() == null ? List.of() : resolveState(sequence.next(), context);
        }
        if (state instanceof States.Action || state instanceof States.DoOnce
                || state instanceof States.WaitSeconds || state instanceof States.WaitUntil
                || state instanceof PathState) {
            return List.of();
        }
        if (state instanceof States.DynamicOutput dynamic) {
            return resolveRequest(outputRequest(null, null, dynamic.output().apply(context)));
        }
        return resolveRequest(outputRequest(control(state), motor(state), output(state)));
    }

    private List<ResolvedOutput> resolveStates(List<State> states, MechanismContext context) {
        List<ResolvedOutput> outputs = new ArrayList<>();
        for (State state : states) {
            outputs.addAll(resolveState(state, context));
        }
        return outputs;
    }

    private List<ResolvedOutput> resolveRequest(OutputRequest request) {
        return request == null ? List.of() : List.of(resolve(request));
    }

    private static OutputRequest outputRequest(ControlBinding control, MotorDevice motor, Output output) {
        if (output == null) {
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

    private static ControlBinding control(State state) {
        if (state instanceof States.ControlPercent controlState) {
            return controlState.control();
        }
        if (state instanceof States.DynamicControlPercent controlState) {
            return controlState.control();
        }
        if (state instanceof States.ControlVoltage controlState) {
            return controlState.control();
        }
        if (state instanceof States.DynamicControlVoltage controlState) {
            return controlState.control();
        }
        if (state instanceof States.ControlPosition controlState) {
            return controlState.control();
        }
        if (state instanceof States.DynamicControlPosition controlState) {
            return controlState.control();
        }
        if (state instanceof States.ControlVelocity controlState) {
            return controlState.control();
        }
        if (state instanceof States.DynamicControlVelocity controlState) {
            return controlState.control();
        }
        return null;
    }

    private static MotorDevice motor(State state) {
        if (state instanceof States.MotorPercent motorState) {
            return motorState.motor();
        }
        if (state instanceof States.DynamicMotorPercent motorState) {
            return motorState.motor();
        }
        if (state instanceof States.MotorVoltage motorState) {
            return motorState.motor();
        }
        if (state instanceof States.DynamicMotorVoltage motorState) {
            return motorState.motor();
        }
        return null;
    }

    private static Output output(State state) {
        return state instanceof Output output ? output : null;
    }
}
