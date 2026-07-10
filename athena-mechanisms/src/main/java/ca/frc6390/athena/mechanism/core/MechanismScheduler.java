package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.MappedActionContext;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

/**
 * Mechanism scheduler used by the root robot runtime.
 */
public final class MechanismScheduler {
    private final MappedActionContext registeredHandles = new MappedActionContext();
    private final ActionContext actionContext;
    private final Map<Mechanism, MechanismRuntime> runtimes = new IdentityHashMap<>();
    private final Map<Action, RequestTarget> actionTargets = new IdentityHashMap<>();
    private final Set<Action> ambiguousActionTargets = Collections.newSetFromMap(new IdentityHashMap<>());
    private final Map<Object, RequestTarget> declarationTargets = new LinkedHashMap<>();
    private final Set<Object> ambiguousDeclarationTargets = new LinkedHashSet<>();
    private final Set<DigitalInputDevice> digitalInputs = new LinkedHashSet<>();
    private final RobotGraph graph = new RobotGraph();
    private final Map<PathAction, PathRuntime> pathRuntimes = new LinkedHashMap<>();
    private final OutputResolver resolver;
    private Runnable simulationStep = () -> {
    };

    private MechanismScheduler(OutputResolver resolver, ActionContext hardwareContext) {
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        actionContext = new OverlayActionContext(registeredHandles,
                hardwareContext == null ? ActionContext.empty() : hardwareContext);
    }

    public static MechanismScheduler create() {
        return new MechanismScheduler(OutputResolver.empty(), ActionContext.empty());
    }

    static MechanismScheduler create(OutputResolver resolver) {
        return new MechanismScheduler(resolver, ActionContext.empty());
    }

    /**
     * Creates a runtime backed by a runtime-owned hardware context.
     *
     * @param hardwareContext hardware context
     * @return mechanism scheduler
     */
    public static MechanismScheduler create(ActionContext hardwareContext) {
        return new MechanismScheduler(OutputResolver.empty(), hardwareContext);
    }

    /**
     * Creates a runtime backed by an output resolver and hardware context.
     *
     * @param resolver output resolver
     * @param hardwareContext hardware context
     * @return mechanism scheduler
     */
    static MechanismScheduler create(OutputResolver resolver, ActionContext hardwareContext) {
        return new MechanismScheduler(resolver, hardwareContext);
    }

    public MechanismScheduler register(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        if (graph.contains(runtimes.keySet(), mechanism)) {
            return this;
        }
        graph.node(mechanism);
        runtimes.computeIfAbsent(mechanism, this::runtime);
        indexActionTargets(mechanism, mechanism, Collections.newSetFromMap(new IdentityHashMap<>()));
        indexDeclarationTargets(mechanism, mechanism, Collections.newSetFromMap(new IdentityHashMap<>()));
        refreshDigitalInputs();
        return this;
    }

    public MechanismScheduler motor(MotorDevice device, MotorHandle motor) {
        MotorDevice safeDevice = Objects.requireNonNull(device, "device");
        registeredHandles.motor(safeDevice, Objects.requireNonNull(motor, "motor"));
        return this;
    }

    public MechanismScheduler encoder(EncoderDevice device, EncoderHandle encoder) {
        registeredHandles.encoder(Objects.requireNonNull(device, "device"), Objects.requireNonNull(encoder, "encoder"));
        return this;
    }

    public MechanismScheduler digitalInput(DigitalInputDevice device, BooleanSupplier reader) {
        DigitalInputDevice safeDevice = Objects.requireNonNull(device, "device");
        DigitalInputDevice.bindRuntime(safeDevice, Objects.requireNonNull(reader, "reader"));
        return this;
    }

    public MechanismScheduler path(PathAction path, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(path, "path"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    public MechanismScheduler paths(Object root, Function<PathAction, PathRuntime> runtimeFactory) {
        Objects.requireNonNull(runtimeFactory, "runtimeFactory");
        for (PathAction path : PathIntrospector.inspect(root)) {
            path(path, runtimeFactory.apply(path));
        }
        return this;
    }

    public MechanismScheduler path(PathAction path, double seconds) {
        return path(path, PathRuntime.timed(seconds));
    }

    public MechanismScheduler simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        for (MechanismRuntime runtime : runtimes.values()) {
            runtime.simulationStep(this.simulationStep);
        }
        return this;
    }

    public MechanismScheduler bindInMemoryRuntime() {
        Set<Object> declarations = graph.declarations(runtimes.keySet());
        for (Object declaration : declarations) {
            if (declaration instanceof MotorDevice motor && !canResolveMotor(motor)) {
                MemoryMotor memory = new MemoryMotor(motor);
                motor(motor, memory);
            } else if (declaration instanceof EncoderDevice encoder && !canResolveEncoder(encoder)) {
                MemoryEncoder memory = new MemoryEncoder(encoder);
                encoder(encoder, memory);
            } else if (declaration instanceof DigitalInputDevice digital) {
                DigitalInputDevice.bindRuntime(digital, () -> false);
            }
        }
        return this;
    }

    public ActionContext actionContext() {
        return actionContext;
    }

    /**
     * Samples declared signal inputs once for the current runtime cycle.
     *
     * @return this runtime
     */
    public MechanismScheduler sampleSignals() {
        digitalInputs.forEach(DigitalInputDevice::sample);
        return this;
    }

    public List<Mechanism> mechanisms() {
        return List.copyOf(runtimes.keySet());
    }

    /**
     * Returns simulation models declared by registered mechanisms.
     *
     * @return simulation models
     */
    public Set<SimModel> simulationModels() {
        return graph.simulations(runtimes.keySet());
    }

    /**
     * Returns declared motors that require hardware follower activation.
     *
     * @return follower motor declarations
     */
    public Set<MotorDevice> followerMotors() {
        Set<MotorDevice> followers = new LinkedHashSet<>();
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof MotorDevice motor && motor.follower() != null) {
                followers.add(motor);
            }
        }
        return Set.copyOf(followers);
    }

    private MechanismScheduler set(Mechanism mechanism, Action action) {
        runtimeFor(mechanism).set(action);
        return this;
    }

    public MechanismScheduler request(Action action) {
        RequestTarget target = targetFor(Objects.requireNonNull(action, "action"));
        if (target == null) {
            throw new IllegalArgumentException("Action is not owned by a registered mechanism and does not target "
                    + "a declaration owned by one.");
        }
        return set(target.root(), action);
    }

    public Action action(Mechanism mechanism) {
        return runtimeFor(mechanism).action();
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        sampleSignals();
        List<ResolvedOutput> outputs = new ArrayList<>();
        for (MechanismRuntime runtime : runtimes.values()) {
            runtime.periodicInto(safeMechanismContext, eventContext, outputs);
        }
        return outputs;
    }

    private void refreshDigitalInputs() {
        digitalInputs.clear();
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof DigitalInputDevice digitalInput) {
                digitalInputs.add(digitalInput);
            }
        }
    }

    public List<ResolvedOutput> robotPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> robotInit(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.INIT, true, false));
    }

    public List<ResolvedOutput> teleopPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> autoPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, true, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> disabledPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, false, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false));
    }

    public List<ResolvedOutput> simulationPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, true),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC, true, true));
    }

    private MechanismRuntime runtime(Mechanism mechanism) {
        return MechanismRuntime.of(graph.node(mechanism), actionContext, resolver, pathRuntimes)
                .simulationStep(simulationStep);
    }

    private MechanismRuntime runtimeFor(Mechanism mechanism) {
        MechanismRuntime runtime = runtimes.get(mechanism);
        if (runtime == null) {
            throw new IllegalArgumentException("Mechanism is not registered: " + mechanism.getClass().getName());
        }
        return runtime;
    }

    private void indexActionTargets(Mechanism root, Mechanism mechanism, Set<Mechanism> visited) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = graph.node(mechanism);
        for (Action action : node.Actions().values()) {
            if (ambiguousActionTargets.contains(action)) {
                continue;
            }
            RequestTarget existing = actionTargets.get(action);
            RequestTarget target = new RequestTarget(root, mechanism);
            if (existing != null && !existing.equals(target)) {
                actionTargets.remove(action);
                ambiguousActionTargets.add(action);
                continue;
            }
            actionTargets.put(action, target);
        }
        for (Mechanism child : node.children().values()) {
            indexActionTargets(root, child, visited);
        }
    }

    private void indexDeclarationTargets(Mechanism root, Mechanism mechanism, Set<Mechanism> visited) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = graph.node(mechanism);
        RequestTarget target = new RequestTarget(root, mechanism);
        for (Object declaration : node.declarations().values()) {
            indexDeclarationTarget(declaration, target);
        }
        for (Mechanism child : node.children().values()) {
            indexDeclarationTargets(root, child, visited);
        }
    }

    private void indexDeclarationTarget(Object declaration, RequestTarget target) {
        if (declaration == null) {
            return;
        }
        if (ambiguousDeclarationTargets.contains(declaration)) {
            return;
        }
        RequestTarget existing = declarationTargets.get(declaration);
        if (existing != null && !existing.equals(target)) {
            declarationTargets.remove(declaration);
            ambiguousDeclarationTargets.add(declaration);
            return;
        }
        declarationTargets.put(declaration, target);
        if (declaration instanceof ControlBinding control) {
            for (Object dependency : controlDeclarations(control)) {
                indexDeclarationTarget(dependency, target);
            }
        }
    }

    private RequestTarget targetFor(Action action) {
        RequestTarget direct = ambiguousActionTargets.contains(action) ? null : actionTargets.get(action);
        if (direct != null) {
            return direct;
        }
        Set<RequestTarget> targets = new HashSet<>();
        for (Object declaration : actionDeclarations(action)) {
            RequestTarget target = declarationTargets.get(declaration);
            if (target != null) {
                targets.add(target);
            }
        }
        if (targets.size() > 1) {
            Mechanism sharedRoot = null;
            for (RequestTarget target : targets) {
                if (sharedRoot == null) {
                    sharedRoot = target.root();
                } else if (sharedRoot != target.root()) {
                    throw new IllegalArgumentException(
                            "Action targets declarations owned by multiple registered mechanism roots.");
                }
            }
            return new RequestTarget(sharedRoot, sharedRoot);
        }
        return targets.isEmpty() ? null : targets.iterator().next();
    }

    private static Set<Object> actionDeclarations(Action action) {
        Set<Object> declarations = new LinkedHashSet<>();
        collectActionDeclarations(action, declarations);
        return declarations;
    }

    private static void collectActionDeclarations(Action action, Set<Object> declarations) {
        if (action == null) {
            return;
        }
        if (action instanceof Actions.Parallel parallel) {
            parallel.Actions().forEach(child -> collectActionDeclarations(child, declarations));
        } else if (action instanceof Actions.Race race) {
            race.Actions().forEach(child -> collectActionDeclarations(child, declarations));
        } else if (action instanceof Actions.Deadline deadline) {
            deadline.Actions().forEach(child -> collectActionDeclarations(child, declarations));
        } else if (action instanceof Actions.Sequence sequence) {
            sequence.steps().forEach(step -> collectActionDeclarations(step.action(), declarations));
            collectActionDeclarations(sequence.next(), declarations);
        } else if (action instanceof Actions.Cycle cycle) {
            cycle.steps().forEach(step -> collectActionDeclarations(step.action(), declarations));
        } else if (action instanceof Actions.Choice choice) {
            collectActionDeclarations(choice.active(), declarations);
            collectActionDeclarations(choice.inactive(), declarations);
        } else if (action instanceof Actions.WhenBranch branch) {
            collectActionDeclarations(branch.active(), declarations);
        } else if (action instanceof Actions.Timeout timeout) {
            collectActionDeclarations(timeout.action(), declarations);
        } else if (action instanceof Actions.Conditional conditional) {
            collectActionDeclarations(conditional.action(), declarations);
            collectActionDeclarations(conditional.next(), declarations);
        } else if (action instanceof Action.Conditional conditional) {
            collectActionDeclarations(conditional.action(), declarations);
            collectActionDeclarations(conditional.next(), declarations);
        } else if (action instanceof Actions.Then then) {
            collectActionDeclarations(then.action(), declarations);
            collectActionDeclarations(then.next(), declarations);
        } else if (action instanceof Action.Then then) {
            collectActionDeclarations(then.action(), declarations);
            collectActionDeclarations(then.next(), declarations);
        } else if (action instanceof Actions.Clamped clamped) {
            collectActionDeclarations(clamped.action(), declarations);
        } else if (action instanceof Action.Clamped clamped) {
            collectActionDeclarations(clamped.action(), declarations);
        } else if (action instanceof PathAction path) {
            declarations.add(path);
        } else if (action instanceof Actions.MotorPercent motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.DynamicMotorPercent motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.MotorVoltage motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.DynamicMotorVoltage motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.EncoderSetPosition encoder) {
            declarations.add(encoder.encoder());
        } else if (control(action) != null) {
            declarations.add(control(action));
            declarations.addAll(controlDeclarations(control(action)));
        }
    }

    private static Set<Object> controlDeclarations(ControlBinding control) {
        Set<Object> declarations = new LinkedHashSet<>();
        declarations.addAll(control.motors());
        declarations.addAll(control.feedback());
        declarations.addAll(control.dependencies());
        for (ControlLoop loop : control.loops()) {
            declarations.addAll(loop.dependencies());
        }
        return declarations;
    }

    private static ControlBinding control(Action action) {
        if (action instanceof Actions.ControlPercent control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlPercent control) {
            return control.control();
        }
        if (action instanceof Actions.ControlVoltage control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlVoltage control) {
            return control.control();
        }
        if (action instanceof Actions.ControlPosition control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlPosition control) {
            return control.control();
        }
        if (action instanceof Actions.ControlVelocity control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlVelocity control) {
            return control.control();
        }
        return null;
    }

    private boolean canResolveMotor(MotorDevice device) {
        try {
            actionContext.motor(device);
            return true;
        } catch (RuntimeException exception) {
            return false;
        }
    }

    private boolean canResolveEncoder(EncoderDevice device) {
        try {
            actionContext.encoder(device);
            return true;
        } catch (RuntimeException exception) {
            return false;
        }
    }

    private record OverlayActionContext(MappedActionContext registered, ActionContext fallback) implements ActionContext {
        private OverlayActionContext {
            registered = Objects.requireNonNull(registered, "registered");
            fallback = Objects.requireNonNull(fallback, "fallback");
        }

        @Override
        public EncoderHandle encoder(EncoderDevice ref) {
            if (registered.hasEncoder(ref)) {
                return registered.encoder(ref);
            }
            return fallback.encoder(ref);
        }

        @Override
        public MotorHandle motor(MotorDevice ref) {
            if (registered.hasMotor(ref)) {
                return registered.motor(ref);
            }
            return fallback.motor(ref);
        }
    }

    private record RequestTarget(Mechanism root, Mechanism owner) {
        private RequestTarget {
            root = Objects.requireNonNull(root, "root");
            owner = Objects.requireNonNull(owner, "owner");
        }
    }

    private static final class MemoryMotor implements MotorHandle {
        private final MotorDevice device;
        private double position;
        private double velocity;

        private MemoryMotor(MotorDevice device) {
            this.device = Objects.requireNonNull(device, "device");
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setPercentOutput(double percent) {
            velocity = percent;
        }

        @Override
        public void setVoltage(double volts) {
            velocity = volts;
        }

        @Override
        public void setPositionTargetRotations(double rotations) {
            position = rotations;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
        }

        @Override
        public double integratedPositionRotations() {
            return position;
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            return velocity;
        }
    }

    private static final class MemoryEncoder implements EncoderHandle {
        private final EncoderDevice device;
        private double position;
        private double velocity;

        private MemoryEncoder(EncoderDevice device) {
            this.device = Objects.requireNonNull(device, "device");
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double absolutePositionRotations() {
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity;
        }

        @Override
        public void setPositionRotations(double rotations) {
            position = Double.isFinite(rotations) ? rotations : 0.0;
        }

        @Override
        public boolean supportsPositionSetting() {
            return true;
        }
    }
}
