package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.MotorRef;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Resolves requested mechanism outputs through the rule system.
 */
public final class OutputResolver {
    private final List<RuleRef> globalRules;

    private OutputResolver(List<RuleRef> globalRules) {
        this.globalRules = List.copyOf(globalRules);
    }

    public static OutputResolver empty() {
        return new OutputResolver(List.of());
    }

    public static OutputResolver of(RuleRef... rules) {
        List<RuleRef> globalRules = new ArrayList<>();
        for (RuleRef rule : rules) {
            globalRules.add(Objects.requireNonNull(rule, "rule"));
        }
        return new OutputResolver(globalRules);
    }

    public ResolvedOutput resolve(OutputRequest request, AxisStateSource axisStates, RuleRef... rules) {
        Objects.requireNonNull(request, "request");
        Objects.requireNonNull(axisStates, "axisStates");
        List<RuleRef> allRules = new ArrayList<>(globalRules);
        if (request.axis() != null) {
            allRules.addAll(request.axis().rules());
        }
        if (request.control() != null) {
            allRules.addAll(request.control().rules());
        }
        for (RuleRef rule : rules) {
            allRules.add(Objects.requireNonNull(rule, "rule"));
        }
        return resolveRules(request, axisStates, allRules);
    }

    public List<ResolvedOutput> resolve(Mechanism mechanism, MechanismState state, AxisStateSource axisStates) {
        return resolve(mechanism, state, MechanismContext.empty(), axisStates);
    }

    public List<ResolvedOutput> resolve(
            Mechanism mechanism,
            MechanismState state,
            MechanismContext context,
            AxisStateSource axisStates) {
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(context, "context");
        Objects.requireNonNull(axisStates, "axisStates");
        return resolveState(List.of(mechanism), state, context, axisStates);
    }

    private List<ResolvedOutput> resolveState(
            List<Mechanism> scope,
            MechanismState state,
            MechanismContext context,
            AxisStateSource axisStates) {
        return resolveState(scope, state, context, axisStates, List.of());
    }

    private List<ResolvedOutput> resolveState(
            List<Mechanism> scope,
            MechanismState state,
            MechanismContext context,
            AxisStateSource axisStates,
            List<RuleRef> stateRules) {
        if (state instanceof States.ChildSet childSet) {
            List<ResolvedOutput> outputs = new ArrayList<>();
            for (States.ChildTarget target : childSet.targets()) {
                List<Mechanism> childScope = new ArrayList<>(scope);
                childScope.add(target.mechanism());
                outputs.addAll(resolveState(childScope, target.state(), context, axisStates, stateRules));
            }
            return outputs;
        }
        if (state instanceof States.Clamped clamped) {
            List<RuleRef> rules = new ArrayList<>(stateRules);
            rules.add(Rules.range(clamped.range()).clamp());
            return resolveState(scope, clamped.state(), context, axisStates, rules);
        }
        if (state instanceof MechanismState.Clamped clamped) {
            List<RuleRef> rules = new ArrayList<>(stateRules);
            rules.add(Rules.range(clamped.range()).clamp());
            return resolveState(scope, clamped.state(), context, axisStates, rules);
        }
        if (state instanceof States.Conditional conditional) {
            return resolveState(scope, conditional.state(), context, axisStates, stateRules);
        }
        if (state instanceof MechanismState.Conditional conditional) {
            return resolveState(scope, conditional.state(), context, axisStates, stateRules);
        }
        if (state instanceof States.Then then) {
            return resolveState(scope, then.state(), context, axisStates, stateRules);
        }
        if (state instanceof MechanismState.Then then) {
            return resolveState(scope, then.state(), context, axisStates, stateRules);
        }
        if (state instanceof States.Timeout timeout) {
            if (timeout.expired(context)) {
                return List.of();
            }
            return resolveState(scope, timeout.state(), context, axisStates, stateRules);
        }
        if (state instanceof States.Choice choice) {
            return resolveState(scope, choice.choose(context), context, axisStates, stateRules);
        }
        if (state instanceof States.WhenBranch branch) {
            return resolveState(scope, branch.choose(context), context, axisStates, stateRules);
        }
        if (state instanceof States.Parallel parallel) {
            return resolveStates(scope, parallel.states(), context, axisStates, stateRules);
        }
        if (state instanceof States.Race race) {
            return resolveStates(scope, race.states(), context, axisStates, stateRules);
        }
        if (state instanceof States.Deadline deadline) {
            return resolveStates(scope, deadline.states(), context, axisStates, stateRules);
        }
        if (state instanceof States.Sequence sequence) {
            if (context.timeInStateSeconds() >= sequence.timeoutSeconds()) {
                return List.of();
            }
            if (!sequence.steps().isEmpty()) {
                return resolveState(scope, sequence.steps().get(0).state(), context, axisStates, stateRules);
            }
            if (sequence.next() != null) {
                return resolveState(scope, sequence.next(), context, axisStates, stateRules);
            }
            return List.of();
        }
        if (state instanceof States.Action || state instanceof States.DoOnce
                || state instanceof States.WaitSeconds || state instanceof States.WaitUntil
                || state instanceof PathRef) {
            return List.of();
        }
        if (state instanceof States.DynamicOutput dynamic) {
            return resolveRequest(scope, outputRequest(null, null, null, dynamic.output().apply(context)), axisStates, stateRules);
        }
        OutputRequest request = outputRequest(control(state), axis(state), motor(state), output(state));
        if (request == null) {
            return List.of();
        }
        return resolveRequest(scope, request, axisStates, stateRules);
    }

    private List<ResolvedOutput> resolveStates(
            List<Mechanism> scope,
            List<MechanismState> states,
            MechanismContext context,
            AxisStateSource axisStates,
            List<RuleRef> stateRules) {
        List<ResolvedOutput> outputs = new ArrayList<>();
        for (MechanismState state : states) {
            outputs.addAll(resolveState(scope, state, context, axisStates, stateRules));
        }
        return outputs;
    }

    private List<ResolvedOutput> resolveRequest(
            List<Mechanism> scope,
            OutputRequest request,
            AxisStateSource axisStates,
            List<RuleRef> stateRules) {
        if (request == null) {
            return List.of();
        }
        List<RuleRef> rules = new ArrayList<>(globalRules);
        if (request.axis() != null) {
            rules.addAll(request.axis().rules());
        }
        if (request.control() != null) {
            rules.addAll(request.control().rules());
        }
        for (Mechanism mechanism : scope) {
            rules.addAll(MechanismIntrospector.inspect(mechanism).rules().values());
        }
        rules.addAll(stateRules);
        return List.of(resolveRules(request, axisStates, rules));
    }

    private ResolvedOutput resolveRules(OutputRequest request, AxisStateSource axisStates, List<RuleRef> rules) {
        Output output = request.output();
        List<RuleResult> results = new ArrayList<>();
        for (RuleRef rule : rules) {
            RuleContext context = ruleContext(axisStates, outputRequest(request.control(), request.axis(), request.motor(), output));
            RuleResult result = rule.evaluate(context);
            results.add(result);
            if (result.decision() == RuleResult.Decision.BLOCK) {
                return new ResolvedOutput(request, blockedOutput(request, axisStates), results);
            }
            if (result.decision() == RuleResult.Decision.CLAMP && result.value().isPresent()) {
                output = withValue(output, result.value().getAsDouble());
            }
        }
        return new ResolvedOutput(request, output, results);
    }

    private static RuleContext ruleContext(AxisStateSource axisStates, OutputRequest request) {
        return new RuleContext() {
            @Override
            public AxisState axis(AxisRef axis) {
                return axisStates.axis(axis);
            }

            @Override
            public OutputRequest request() {
                return request;
            }
        };
    }

    private static OutputRequest outputRequest(ControlRef control, AxisRef axis, MotorRef motor, Output output) {
        if (output == null) {
            return null;
        }
        if (control != null) {
            return OutputRequest.of(control, output);
        }
        if (axis != null) {
            return OutputRequest.of(axis, output);
        }
        if (motor != null) {
            return OutputRequest.of(motor, output);
        }
        return OutputRequest.of((AxisRef) null, output);
    }

    private static ControlRef control(MechanismState state) {
        if (state instanceof States.ControlPercent loopState) {
            return loopState.control();
        }
        if (state instanceof States.DynamicControlPercent loopState) {
            return loopState.control();
        }
        if (state instanceof States.ControlVoltage loopState) {
            return loopState.control();
        }
        if (state instanceof States.DynamicControlVoltage loopState) {
            return loopState.control();
        }
        if (state instanceof States.ControlPosition loopState) {
            return loopState.control();
        }
        if (state instanceof States.DynamicControlPosition loopState) {
            return loopState.control();
        }
        if (state instanceof States.ControlVelocity loopState) {
            return loopState.control();
        }
        if (state instanceof States.DynamicControlVelocity loopState) {
            return loopState.control();
        }
        return null;
    }

    private static AxisRef axis(MechanismState state) {
        if (state instanceof States.AxisPercent axisState) {
            return axisState.axis();
        }
        if (state instanceof States.DynamicAxisPercent axisState) {
            return axisState.axis();
        }
        if (state instanceof States.AxisVoltage axisState) {
            return axisState.axis();
        }
        if (state instanceof States.DynamicAxisVoltage axisState) {
            return axisState.axis();
        }
        if (state instanceof States.AxisPosition axisState) {
            return axisState.axis();
        }
        if (state instanceof States.DynamicAxisPosition axisState) {
            return axisState.axis();
        }
        if (state instanceof States.AxisVelocity axisState) {
            return axisState.axis();
        }
        if (state instanceof States.DynamicAxisVelocity axisState) {
            return axisState.axis();
        }
        return null;
    }

    private static MotorRef motor(MechanismState state) {
        if (state instanceof MotorRef.MotorState motorState) {
            return motorState.motor();
        }
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

    private static Output output(MechanismState state) {
        if (state instanceof Output output) {
            return output;
        }
        return null;
    }

    private static Output withValue(Output output, double value) {
        if (output instanceof Output.Percent) {
            return Outputs.percent(value);
        }
        if (output instanceof Output.Voltage) {
            return Outputs.voltage(value);
        }
        if (output instanceof Output.Position) {
            return Outputs.position(value);
        }
        if (output instanceof Output.Velocity) {
            return Outputs.velocity(value);
        }
        return output;
    }

    private static Output blockedOutput(OutputRequest request, AxisStateSource axisStates) {
        AxisRef axis = request.axis();
        if (axis == null) {
            return Outputs.neutral();
        }
        AxisState state = axisStates.axis(axis);
        return switch (axis.blockPolicy()) {
            case HOLD_POSITION -> Outputs.position(state.position());
            case HOLD_VELOCITY -> Outputs.velocity(state.velocity());
            case NEUTRAL -> Outputs.neutral();
        };
    }
}
