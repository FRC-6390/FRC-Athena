package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.runtime.MappedActionContext;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import java.util.ArrayList;
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
 * Robot-level runtime for Athena mechanisms.
 */
public final class RobotRuntime {
    private final MappedActionContext registeredHandles = new MappedActionContext();
    private final ActionContext actionContext;
    private final Map<Mechanism, MechanismRuntime> runtimes = new IdentityHashMap<>();
    private final RobotGraph graph = new RobotGraph();
    private final Map<MotorDevice, SimMotor> simMotors = new LinkedHashMap<>();
    private final Map<EncoderDevice, SimEncoder> simEncoders = new LinkedHashMap<>();
    private final Map<DigitalInputDevice, SimDigitalInput> digitalInputs = new LinkedHashMap<>();
    private final Map<PathState, PathRuntime> pathRuntimes = new LinkedHashMap<>();
    private final Set<SimModel> simulations = new LinkedHashSet<>();
    private final SimModelRunner simModelRunner = new SimModelRunner();
    private final OutputResolver resolver;
    private Runnable simulationStep = () -> {
    };

    private RobotRuntime(OutputResolver resolver, ActionContext hardwareContext) {
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        actionContext = new OverlayActionContext(registeredHandles,
                hardwareContext == null ? ActionContext.empty() : hardwareContext);
    }

    public static RobotRuntime create() {
        return new RobotRuntime(OutputResolver.empty(), ActionContext.empty());
    }

    public static RobotRuntime create(OutputResolver resolver) {
        return new RobotRuntime(resolver, ActionContext.empty());
    }

    /**
     * Creates a runtime backed by a runtime-owned hardware context.
     *
     * @param hardwareContext hardware context
     * @return robot runtime
     */
    public static RobotRuntime create(ActionContext hardwareContext) {
        return new RobotRuntime(OutputResolver.empty(), hardwareContext);
    }

    /**
     * Creates a runtime backed by an output resolver and hardware context.
     *
     * @param resolver output resolver
     * @param hardwareContext hardware context
     * @return robot runtime
     */
    public static RobotRuntime create(OutputResolver resolver, ActionContext hardwareContext) {
        return new RobotRuntime(resolver, hardwareContext);
    }

    public RobotRuntime register(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        graph.node(mechanism);
        runtimes.computeIfAbsent(mechanism, this::runtime);
        refreshSimulations();
        return this;
    }

    public RobotRuntime motor(MotorDevice device, MotorHandle motor) {
        MotorDevice safeDevice = Objects.requireNonNull(device, "device");
        SimMotor simulated = new SimMotor(motor);
        simMotors.put(safeDevice, simulated);
        registeredHandles.motor(safeDevice, simulated);
        return this;
    }

    public RobotRuntime encoder(EncoderDevice device, EncoderHandle encoder) {
        SimEncoder simulated = new SimEncoder(encoder);
        simEncoders.put(Objects.requireNonNull(device, "device"), simulated);
        registeredHandles.encoder(device, simulated);
        return this;
    }

    public RobotRuntime digitalInput(DigitalInputDevice device, BooleanSupplier reader) {
        DigitalInputDevice safeDevice = Objects.requireNonNull(device, "device");
        DigitalInputDevice.bindRuntime(safeDevice, Objects.requireNonNull(reader, "reader"));
        return this;
    }

    private RobotRuntime simulatedDigitalInput(DigitalInputDevice device, SimDigitalInput value) {
        digitalInputs.put(Objects.requireNonNull(device, "device"), Objects.requireNonNull(value, "value"));
        DigitalInputDevice.bindRuntime(device, value::get);
        return this;
    }

    public RobotRuntime path(PathState path, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(path, "path"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    public RobotRuntime paths(Object root, Function<PathState, PathRuntime> runtimeFactory) {
        Objects.requireNonNull(runtimeFactory, "runtimeFactory");
        for (PathState path : PathIntrospector.inspect(root)) {
            path(path, runtimeFactory.apply(path));
        }
        return this;
    }

    public RobotRuntime path(PathState path, double seconds) {
        return path(path, PathRuntime.timed(seconds));
    }

    public RobotRuntime simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        for (MechanismRuntime runtime : runtimes.values()) {
            runtime.simulationStep(this.simulationStep);
        }
        return this;
    }

    public RobotRuntime bindInMemoryRuntime() {
        Set<Object> declarations = graph.declarations(runtimes.keySet());
        for (Object declaration : declarations) {
            if (declaration instanceof MotorDevice motor && !canResolveMotor(motor)) {
                MemoryMotor memory = new MemoryMotor(motor);
                motor(motor, memory);
            } else if (declaration instanceof EncoderDevice encoder && !canResolveEncoder(encoder)) {
                MemoryEncoder memory = new MemoryEncoder(encoder);
                encoder(encoder, memory);
            } else if (declaration instanceof DigitalInputDevice digital && !digitalInputs.containsKey(digital)) {
                simulatedDigitalInput(digital, new SimDigitalInput());
            }
        }
        refreshSimulations();
        return this;
    }

    public ActionContext actionContext() {
        return actionContext;
    }

    public List<Mechanism> mechanisms() {
        return List.copyOf(runtimes.keySet());
    }

    public RobotRuntime set(Mechanism mechanism, State state) {
        runtimeFor(mechanism).set(state);
        return this;
    }

    public State state(Mechanism mechanism) {
        return runtimeFor(mechanism).state();
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        List<ResolvedOutput> outputs = new ArrayList<>();
        for (MechanismRuntime runtime : runtimes.values()) {
            outputs.addAll(runtime.periodic(safeMechanismContext, eventContext));
        }
        if (safeMechanismContext.simulation()) {
            simModelRunner.step(simulations, simMotors, simEncoders, digitalInputs, safeMechanismContext.dtSeconds());
        }
        return outputs;
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

    private void refreshSimulations() {
        simulations.clear();
        simulations.addAll(graph.simulations(runtimes.keySet()));
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
    }
}
