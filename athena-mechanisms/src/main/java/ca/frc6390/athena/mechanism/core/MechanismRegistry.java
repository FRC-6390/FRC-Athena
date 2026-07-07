package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.ref.ActionContext;
import ca.frc6390.athena.hardware.ref.BooleanRef;
import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.MappedActionContext;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.NumberRef;
import ca.frc6390.athena.hardware.ref.RuntimeBoolean;
import ca.frc6390.athena.hardware.ref.RuntimeEncoder;
import ca.frc6390.athena.hardware.ref.RuntimeMotor;
import ca.frc6390.athena.hardware.ref.RuntimeNumber;
import ca.frc6390.athena.hardware.ref.SimLimitRef;
import ca.frc6390.athena.hardware.ref.SimRef;
import java.util.Collections;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.IdentityHashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;

/**
 * Robot-level registration and periodic runner for Athena mechanisms.
 */
public final class MechanismRegistry {
    private final MappedActionContext actionContext = new MappedActionContext();
    private final Map<Mechanism, MechanismRuntime> runtimes = new IdentityHashMap<>();
    private final Map<MotorRef, SimMotor> simMotors = new LinkedHashMap<>();
    private final Map<EncoderRef, SimEncoder> simEncoders = new LinkedHashMap<>();
    private final Map<DigitalInputRef, RuntimeBoolean> digitalInputs = new LinkedHashMap<>();
    private final Map<PathRef, PathRuntime> pathRuntimes = new LinkedHashMap<>();
    private final Set<SimRef> simulations = new LinkedHashSet<>();
    private final SimModelRunner simModelRunner = new SimModelRunner();
    private final OutputResolver resolver;
    private Runnable simulationStep = () -> {
    };

    private MechanismRegistry(OutputResolver resolver) {
        this.resolver = Objects.requireNonNull(resolver, "resolver");
    }

    public static MechanismRegistry create() {
        return new MechanismRegistry(OutputResolver.empty());
    }

    public static MechanismRegistry create(OutputResolver resolver) {
        return new MechanismRegistry(resolver);
    }

    public MechanismRegistry register(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        runtimes.computeIfAbsent(mechanism, this::runtime);
        refreshSimulations();
        return this;
    }

    public MechanismRegistry motor(MotorRef ref, RuntimeMotor motor) {
        MotorRef safeRef = Objects.requireNonNull(ref, "ref");
        SimMotor simulated = new SimMotor(motor);
        simMotors.put(safeRef, simulated);
        actionContext.motor(safeRef, simulated);
        return this;
    }

    public MechanismRegistry motor(MotorRef ref, MotorDevice motor) {
        return motor(ref, RuntimeMotor.from(motor));
    }

    public MechanismRegistry encoder(EncoderRef ref, RuntimeEncoder encoder) {
        SimEncoder simulated = new SimEncoder(encoder);
        simEncoders.put(ref, simulated);
        actionContext.encoder(ref, simulated);
        return this;
    }

    public MechanismRegistry bool(BooleanRef ref, RuntimeBoolean value) {
        actionContext.bool(ref, value);
        return this;
    }

    public MechanismRegistry digitalInput(DigitalInputRef ref, RuntimeBoolean value) {
        digitalInputs.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(value, "value"));
        DigitalInputRef.bindRuntime(ref, value::get);
        return this;
    }

    public MechanismRegistry number(NumberRef ref, RuntimeNumber value) {
        actionContext.number(ref, value);
        return this;
    }

    public MechanismRegistry path(PathRef ref, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    public MechanismRegistry paths(Object root, Function<PathRef, PathRuntime> runtimeFactory) {
        Objects.requireNonNull(runtimeFactory, "runtimeFactory");
        for (PathRef ref : PathIntrospector.inspect(root)) {
            path(ref, runtimeFactory.apply(ref));
        }
        return this;
    }

    public MechanismRegistry path(PathRef ref, double seconds) {
        return path(ref, PathRuntime.timed(seconds));
    }

    public MechanismRegistry simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        for (MechanismRuntime runtime : runtimes.values()) {
            runtime.simulationStep(this.simulationStep);
        }
        return this;
    }

    public MechanismRegistry bindInMemoryRuntime() {
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        Set<Object> refs = new HashSet<>();
        for (Mechanism mechanism : runtimes.keySet()) {
            collectRefs(mechanism, visited, refs);
        }
        for (Object ref : refs) {
            if (ref instanceof MotorRef motor && !actionContext.hasMotor(motor)) {
                actionContext.motor(motor, new MemoryMotor());
                simMotors.put(motor, new SimMotor(actionContext.motor(motor)));
                actionContext.motor(motor, simMotors.get(motor));
            } else if (ref instanceof EncoderRef encoder && !actionContext.hasEncoder(encoder)) {
                actionContext.encoder(encoder, new MemoryEncoder());
                simEncoders.put(encoder, new SimEncoder(actionContext.encoder(encoder)));
                actionContext.encoder(encoder, simEncoders.get(encoder));
            } else if (ref instanceof BooleanRef bool && !actionContext.hasBoolean(bool)) {
                actionContext.bool(bool, new MemoryBoolean());
            } else if (ref instanceof DigitalInputRef digital && !digitalInputs.containsKey(digital)) {
                digitalInput(digital, new MemoryBoolean());
            } else if (ref instanceof NumberRef number && !actionContext.hasNumber(number)) {
                actionContext.number(number, new MemoryNumber());
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

    public MechanismRegistry set(Mechanism mechanism, MechanismState state) {
        runtimeFor(mechanism).set(state);
        return this;
    }

    public MechanismState state(Mechanism mechanism) {
        return runtimeFor(mechanism).state();
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        List<ResolvedOutput> outputs = new ArrayList<>();
        for (MechanismRuntime runtime : runtimes.values()) {
            outputs.addAll(runtime.periodic(safeMechanismContext, eventContext));
        }
        if (safeMechanismContext.simulation()) {
            simModelRunner.step(
                    simulations,
                    simMotors,
                    simEncoders,
                    digitalInputs,
                    safeMechanismContext.dtSeconds());
        }
        return outputs;
    }

    public List<ResolvedOutput> robotPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> robotInit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.INIT, true, false));
    }

    public List<ResolvedOutput> robotExit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.EXIT, true, false));
    }

    public List<ResolvedOutput> teleopPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> teleopInit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.TELEOP, LifecyclePhase.INIT, true, false));
    }

    public List<ResolvedOutput> teleopExit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.TELEOP, LifecyclePhase.EXIT, true, false));
    }

    public List<ResolvedOutput> autoPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, true, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> autoInit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, true, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, false));
    }

    public List<ResolvedOutput> autoExit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, true, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT, true, false));
    }

    public List<ResolvedOutput> disabledPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, false, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false));
    }

    public List<ResolvedOutput> disabledInit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, false, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.DISABLED, LifecyclePhase.INIT, false, false));
    }

    public List<ResolvedOutput> disabledExit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, false, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.DISABLED, LifecyclePhase.EXIT, false, false));
    }

    public List<ResolvedOutput> simulationPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, true),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC, true, true));
    }

    public List<ResolvedOutput> simulationInit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, true),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.SIMULATION, LifecyclePhase.INIT, true, true));
    }

    public List<ResolvedOutput> simulationExit(double nowSeconds, double dtSeconds) {
        return periodic(
                new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, true),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.SIMULATION, LifecyclePhase.EXIT, true, true));
    }

    private MechanismRuntime runtime(Mechanism mechanism) {
        return MechanismRuntime.of(mechanism, actionContext, resolver, AxisStateSource.from(actionContext), pathRuntimes)
                .simulationStep(simulationStep);
    }

    private MechanismRuntime runtimeFor(Mechanism mechanism) {
        MechanismRuntime runtime = runtimes.get(mechanism);
        if (runtime == null) {
            throw new IllegalArgumentException("Mechanism is not registered: " + mechanism.getClass().getName());
        }
        return runtime;
    }

    private static void collectRefs(Mechanism mechanism, Set<Mechanism> visited, Set<Object> refs) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismDefinition definition = MechanismIntrospector.inspect(mechanism);
        for (Object ref : definition.refs().values()) {
            refs.add(ref);
            if (ref instanceof AxisRef axis) {
                refs.addAll(axis.motors());
                refs.addAll(axis.encoders());
                refs.addAll(axis.sensors());
                refs.addAll(axis.refs());
                for (ControlLoopRef loop : axis.loops()) {
                    refs.addAll(loop.refs());
                }
            } else if (ref instanceof ControlRef control) {
                refs.addAll(control.motors());
                refs.addAll(control.feedback());
                refs.addAll(control.refs());
                for (ControlLoopRef loop : control.loops()) {
                    refs.addAll(loop.refs());
                }
            } else if (ref instanceof SimRef simulation) {
                collectSimRefs(simulation, refs);
            }
        }
        for (Mechanism child : definition.children().values()) {
            collectRefs(child, visited, refs);
        }
    }

    private void refreshSimulations() {
        simulations.clear();
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        Set<Object> refs = new HashSet<>();
        for (Mechanism mechanism : runtimes.keySet()) {
            collectRefs(mechanism, visited, refs);
        }
        for (Object ref : refs) {
            if (ref instanceof SimRef simulation) {
                simulations.add(simulation);
            }
        }
    }

    private static void collectSimRefs(SimRef simulation, Set<Object> refs) {
        refs.addAll(simulation.motors());
        refs.addAll(simulation.encoders());
        for (MotorRef motor : simulation.motors()) {
            refs.add(motor.encoder());
        }
        for (Object ref : simulation.refs()) {
            refs.add(ref);
            if (ref instanceof AxisRef axis) {
                refs.addAll(axis.motors());
                refs.addAll(axis.encoders());
                refs.addAll(axis.sensors());
                refs.addAll(axis.refs());
                for (ControlLoopRef loop : axis.loops()) {
                    refs.addAll(loop.refs());
                }
            } else if (ref instanceof ControlRef control) {
                refs.addAll(control.motors());
                refs.addAll(control.feedback());
                refs.addAll(control.refs());
                for (ControlLoopRef loop : control.loops()) {
                    refs.addAll(loop.refs());
                }
            } else if (ref instanceof SimLimitRef limit) {
                refs.add(limit.sensor());
            }
        }
    }

    private static final class MemoryMotor implements RuntimeMotor {
        private double position;
        private double velocity;

        @Override
        public void percent(double output) {
            velocity = output;
        }

        @Override
        public void voltage(double volts) {
            velocity = volts;
        }

        @Override
        public void position(double position) {
            this.position = position;
        }

        @Override
        public void velocity(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public void brake() {
        }

        @Override
        public void coast() {
        }
    }

    private static final class MemoryEncoder implements RuntimeEncoder {
        private double position;

        @Override
        public double position() {
            return position;
        }

        @Override
        public double absolutePosition() {
            return position;
        }

        @Override
        public double velocity() {
            return 0.0;
        }

        @Override
        public void set(double position) {
            this.position = position;
        }
    }

    private static final class MemoryBoolean implements RuntimeBoolean {
        private boolean value;

        @Override
        public boolean get() {
            return value;
        }

        @Override
        public void set(boolean value) {
            this.value = value;
        }
    }

    private static final class MemoryNumber implements RuntimeNumber {
        private double value;

        @Override
        public double get() {
            return value;
        }

        @Override
        public void set(double value) {
            this.value = value;
        }
    }
}
