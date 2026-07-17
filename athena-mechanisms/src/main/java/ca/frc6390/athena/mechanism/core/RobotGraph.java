package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import ca.frc6390.athena.mechanism.sysid.ControlSysId;

final class RobotGraph {
    private final Map<Mechanism, MechanismNode> mechanismNodes = new IdentityHashMap<>();

    MechanismNode node(Mechanism mechanism) {
        return mechanismNodes.computeIfAbsent(
                Objects.requireNonNull(mechanism, "mechanism"),
                MechanismIntrospector::inspect);
    }

    boolean contains(Collection<Mechanism> roots, Mechanism target) {
        Objects.requireNonNull(target, "target");
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Mechanism root : roots) {
            if (contains(root, target, visited)) {
                return true;
            }
        }
        return false;
    }

    Set<Object> declarations(Collection<Mechanism> mechanisms) {
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        Set<Object> declarations = new HashSet<>();
        for (Mechanism mechanism : mechanisms) {
            collectDeclarations(mechanism, visited, declarations);
        }
        return declarations;
    }

    Set<SimModel> simulations(Collection<Mechanism> mechanisms) {
        Set<SimModel> simulations = new LinkedHashSet<>();
        for (Object declaration : declarations(mechanisms)) {
            if (declaration instanceof SimModel simulation) {
                simulations.add(simulation);
            } else if (declaration instanceof SimModel.Source source) {
                SimModel simulation = source.simulationModel();
                if (simulation != null) {
                    simulations.add(simulation);
                }
            }
        }
        return simulations;
    }

    TelemetrySchema telemetrySchema(
            Collection<Mechanism> mechanisms,
            RuntimeOverrides overrides,
            MechanismScheduler scheduler) {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        // A declaration referenced by a control or TelemetrySource must not also be published
        // as a second writable topic. Otherwise each NT entry can overwrite the same runtime
        // value in turn, making dashboard changes appear to revert immediately.
        Set<Object> directDeclarations = directDeclarations(mechanisms);
        Set<Object> publishedDeclarations = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Mechanism mechanism : mechanisms) {
            collectTelemetry(node(mechanism).name(), mechanism, visited, values, overrides,
                    directDeclarations, publishedDeclarations);
        }
        MutableTelemetryNode root = new MutableTelemetryNode("", TelemetryNode.Kind.ROOT);
        visited.clear();
        for (Mechanism mechanism : mechanisms) {
            addMechanismSchema(root, node(mechanism).name(), mechanism, visited, scheduler, overrides);
        }
        values.forEach((path, value) -> root.value(path, value));
        return new TelemetrySchema(root.freeze());
    }

    private void addMechanismSchema(MutableTelemetryNode parent, String name, Mechanism mechanism,
            Set<Mechanism> visited, MechanismScheduler scheduler, RuntimeOverrides overrides) {
        if (!visited.add(mechanism)) return;
        MechanismNode inspected = node(mechanism);
        String[] nameParts = name.split("/");
        MutableTelemetryNode mechanismNode = parent;
        for (int index = 0; index < nameParts.length; index++) {
            mechanismNode = mechanismNode.child(nameParts[index], index == nameParts.length - 1
                    ? TelemetryNode.Kind.MECHANISM : TelemetryNode.Kind.GROUP);
        }
        final MutableTelemetryNode ownedNode = mechanismNode;
        for (TelemetrySchema.Group group : TelemetrySchema.Group.values()) {
            ownedNode.child(group.path(), TelemetryNode.Kind.GROUP);
        }
        MutableTelemetryNode state = ownedNode.child(TelemetrySchema.Group.STATE.path(), TelemetryNode.Kind.GROUP);
        state.values.put("ActiveAction", TelemetryValue.string(() -> activeActionName(inspected, scheduler)));
        state.values.put("ActionRunning", TelemetryValue.bool(() -> anyAction(inspected, scheduler, true)));
        state.values.put("ActionComplete", TelemetryValue.bool(() -> anyAction(inspected, scheduler, false)));
        inspected.Actions().forEach((actionName, action) -> {
            if (!isExposedActionField(mechanism, actionName, action)) return;
            ownedNode
                .child(TelemetrySchema.Group.ACTIONS.path(), TelemetryNode.Kind.GROUP)
                .actions.put(actionName, new TelemetryAction(actionName, action,
                        () -> requireScheduler(scheduler).request(action),
                        () -> requireScheduler(scheduler).cancel(action),
                        () -> scheduler != null && scheduler.isRunning(action),
                        () -> scheduler != null && scheduler.isComplete(action)));
        });
        inspected.declarations().forEach((declarationName, declaration) -> {
            if (!(declaration instanceof ControlBinding control) || control.output() == null) return;
            MutableTelemetryNode testNode = ownedNode
                    .child(TelemetrySchema.Group.CONTROLS.path(), TelemetryNode.Kind.GROUP)
                    .child(declarationName, TelemetryNode.Kind.GROUP)
                    .child("Test", TelemetryNode.Kind.GROUP);
            Action percentAction = control.percent(() -> overrides.testPercent(control));
            testNode.actions.put("RunPercent", new TelemetryAction(
                    "RunPercent", percentAction,
                    () -> requireScheduler(scheduler).request(percentAction),
                    () -> requireScheduler(scheduler).cancel(percentAction),
                    () -> scheduler != null && scheduler.isRunning(percentAction),
                    () -> scheduler != null && scheduler.isComplete(percentAction)));
        });
        inspected.children().forEach((childName, child) ->
                addMechanismSchema(ownedNode, childName, child, visited, scheduler, overrides));
    }

    private static boolean isExposedActionField(Mechanism mechanism, String name, Action action) {
        Class<?> type = mechanism.getClass();
        while (type != null && type != Object.class) {
            for (Field field : type.getDeclaredFields()) {
                if (ControlSysId.class.isAssignableFrom(field.getType())
                        && action instanceof Actions.ControlSysIdAction sysIdAction) {
                    try {
                        if (!field.canAccess(Modifier.isStatic(field.getModifiers()) ? null : mechanism)) {
                            field.setAccessible(true);
                        }
                        Object value = field.get(Modifier.isStatic(field.getModifiers()) ? null : mechanism);
                        if (value != sysIdAction.routine()) continue;
                        Telemetry annotation = field.getAnnotation(Telemetry.class);
                        String prefix = annotation == null
                                ? field.getName() : AnnotatedTelemetry.name(field.getName(), annotation);
                        return name.startsWith(prefix + "/")
                                && (Modifier.isPublic(field.getModifiers()) || annotation != null);
                    } catch (IllegalAccessException exception) {
                        throw new IllegalStateException("Unable to inspect SysId field " + field.getName(), exception);
                    }
                }
                if (!Action.class.isAssignableFrom(field.getType())) continue;
                try {
                    if (!field.canAccess(Modifier.isStatic(field.getModifiers()) ? null : mechanism)) {
                        field.setAccessible(true);
                    }
                    Object value = field.get(Modifier.isStatic(field.getModifiers()) ? null : mechanism);
                    if (value != action) continue;
                    Telemetry annotation = field.getAnnotation(Telemetry.class);
                    String publishedName = annotation == null
                            ? field.getName() : AnnotatedTelemetry.name(field.getName(), annotation);
                    return publishedName.equals(name)
                            && (Modifier.isPublic(field.getModifiers()) || annotation != null);
                } catch (IllegalAccessException exception) {
                    throw new IllegalStateException("Unable to inspect action field " + field.getName(), exception);
                }
            }
            type = type.getSuperclass();
        }
        return false;
    }

    private static MechanismScheduler requireScheduler(MechanismScheduler scheduler) {
        if (scheduler == null) throw new IllegalStateException("Telemetry action is not attached to a scheduler.");
        return scheduler;
    }

    private static String activeActionName(MechanismNode node, MechanismScheduler scheduler) {
        if (scheduler == null) return "";
        return node.Actions().entrySet().stream()
                .filter(entry -> scheduler.isRunning(entry.getValue()))
                .map(Map.Entry::getKey)
                .findFirst()
                .orElse("");
    }

    private static boolean anyAction(MechanismNode node, MechanismScheduler scheduler, boolean running) {
        if (scheduler == null) return false;
        return node.Actions().values().stream().anyMatch(action ->
                running ? scheduler.isRunning(action) : scheduler.isComplete(action));
    }

    private Set<Object> directDeclarations(Collection<Mechanism> mechanisms) {
        Set<Object> declarations = Collections.newSetFromMap(new IdentityHashMap<>());
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Mechanism mechanism : mechanisms) {
            collectDirectDeclarations(mechanism, visited, declarations);
        }
        return declarations;
    }

    private void collectDirectDeclarations(
            Mechanism mechanism,
            Set<Mechanism> visited,
            Set<Object> declarations) {
        if (!visited.add(mechanism)) return;
        MechanismNode node = node(mechanism);
        node.declarations().values().forEach(declaration -> addDirectDeclaration(declaration, declarations));
        node.children().values().forEach(child -> collectDirectDeclarations(child, visited, declarations));
    }

    private static void addDirectDeclaration(Object declaration, Set<Object> declarations) {
        if (declaration instanceof Iterable<?> nested) {
            nested.forEach(value -> addDirectDeclaration(value, declarations));
            return;
        }
        declarations.add(declaration);
    }

    private void collectTelemetry(
            String path,
            Mechanism mechanism,
            Set<Mechanism> visited,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides,
            Set<Object> directDeclarations,
            Set<Object> publishedDeclarations) {
        if (!visited.add(mechanism)) return;
        MechanismNode node = node(mechanism);
        node.declarations().forEach((name, declaration) ->
                addTelemetry(path, name, declaration, values, overrides,
                        directDeclarations, publishedDeclarations, false));
        node.children().forEach((name, child) ->
                collectTelemetry(path + "/" + name, child, visited, values, overrides,
                        directDeclarations, publishedDeclarations));
    }

    private static void addTelemetry(
            String mechanismPath,
            String valuePath,
            Object declaration,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides,
            Set<Object> directDeclarations,
            Set<Object> publishedDeclarations,
            boolean referenced) {
        if ((referenced && directDeclarations.contains(declaration))
                || !publishedDeclarations.add(declaration)) {
            return;
        }
        if (declaration instanceof TelemetryValue value) {
            values.put(mechanismPath + "/Values/" + valuePath, value);
        } else if (declaration instanceof MotorDevice motor) {
            addMotorTelemetry(mechanismPath + "/Devices/" + valuePath, motor, values, overrides);
        } else if (declaration instanceof EncoderDevice encoder) {
            addEncoderTelemetry(mechanismPath + "/Devices/" + valuePath, encoder, values, overrides);
        } else if (declaration instanceof ImuDevice imu) {
            addImuTelemetry(mechanismPath + "/Devices/" + valuePath, imu, values, overrides);
        } else if (declaration instanceof DigitalInputDevice input) {
            String inputPath = mechanismPath + "/Devices/" + valuePath + "/State";
            values.put(inputPath + "/Active", TelemetryValue.bool(input::active));
            values.put(inputPath + "/Raw", TelemetryValue.bool(input::raw));
            String inputInfo = mechanismPath + "/Devices/" + valuePath + "/Info";
            values.put(inputInfo + "/Type", TelemetryValue.constant("DigitalInput"));
            values.put(inputInfo + "/Channel", TelemetryValue.constant(input.channel()));
            values.put(inputInfo + "/DeclaredInverted", TelemetryValue.bool(input::isInverted));
            values.put(mechanismPath + "/Devices/" + valuePath + "/Setup/ClearEdges",
                    TelemetryValue.command(input::clearLatchedEdges));
        } else if (declaration instanceof ControlBinding control) {
            String controlPath = mechanismPath + "/Controls/" + valuePath;
            values.put(controlPath + "/Config/Disabled", overrides.controlDisabled(control));
            values.put(controlPath + "/State/Mode", TelemetryValue.string(() -> control.mode().name()));
            values.put(controlPath + "/Config/Slot", TelemetryValue.number(() -> control.slot()));
            values.put(controlPath + "/Config/Constraints/MinimumPosition", TelemetryValue.writableNumber(
                    () -> overrides.tuning(control).minimumPosition(),
                    value -> overrides.minimumPosition(control, value)));
            values.put(controlPath + "/Config/Constraints/MaximumPosition", TelemetryValue.writableNumber(
                    () -> overrides.tuning(control).maximumPosition(),
                    value -> overrides.maximumPosition(control, value)));
            values.put(controlPath + "/Config/Constraints/MaximumVelocity", TelemetryValue.writableNumber(
                    () -> overrides.tuning(control).maxVelocity(),
                    value -> overrides.maximumVelocity(control, value)));
            values.put(controlPath + "/Config/Constraints/MaximumAcceleration", TelemetryValue.writableNumber(
                    () -> overrides.tuning(control).maxAcceleration(),
                    value -> overrides.maximumAcceleration(control, value)));
            values.put(controlPath + "/Config/Restore", TelemetryValue.command(() -> overrides.restore(control)));
            values.put(controlPath + "/Test/Percent", TelemetryValue.writableNumber(
                    () -> overrides.testPercent(control), value -> overrides.testPercent(control, value)));
            values.put(controlPath + "/Info/Type", TelemetryValue.constant("Control"));
            String motorNames = control.motors().stream()
                    .map(MotorDevice::defaultName)
                    .collect(java.util.stream.Collectors.joining(","));
            values.put(controlPath + "/Info/Motors", TelemetryValue.constant(motorNames));
            for (int index = 0; index < control.motors().size(); index++) {
                addMotorTelemetry(
                        controlPath + "/Devices/" + index,
                        control.motors().get(index),
                        values,
                        overrides,
                        directDeclarations,
                        publishedDeclarations);
            }
            if (control.feedback() != null) {
                values.put(controlPath + "/State/FeedbackPosition",
                        safeNumber(control.feedback().position()::position));
                values.put(controlPath + "/State/FeedbackVelocity",
                        safeNumber(control.feedback().velocity()::velocity));
            }
            int customLoop = 0;
            for (ControlLoop loop : control.loops()) {
                if (directDeclarations.contains(loop) || !publishedDeclarations.add(loop)) continue;
                String loopName = loop instanceof ca.frc6390.athena.mechanism.control.PidGains ? "PID"
                        : loop instanceof ca.frc6390.athena.mechanism.control.FeedforwardGains ? "Feedforward"
                        : "Loop" + customLoop++;
                if (loop instanceof TelemetrySource source) {
                    source.telemetry().forEach((name, value) ->
                            values.put(controlPath + "/Config/" + loopName + "/" + name, value));
                }
            }
        } else if (declaration instanceof TelemetrySource source) {
            source.telemetry().forEach((name, value) -> addTelemetry(
                    mechanismPath, valuePath + "/" + name, value, values, overrides,
                    directDeclarations, publishedDeclarations, true));
        } else if (declaration instanceof Iterable<?> nested) {
            int index = 0;
            for (Object value : nested) {
                addTelemetry(mechanismPath, valuePath + "/" + index++, value, values, overrides,
                        directDeclarations, publishedDeclarations, false);
            }
        }
    }

    private static void addMotorTelemetry(
            String path,
            MotorDevice motor,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides,
            Set<Object> directDeclarations,
            Set<Object> publishedDeclarations) {
        if (directDeclarations.contains(motor) || !publishedDeclarations.add(motor)) return;
        addMotorTelemetry(path, motor, values, overrides);
    }

    private static void addMotorTelemetry(
            String path,
            MotorDevice motor,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides) {
        String state = path + "/State";
        String config = path + "/Config";
        String info = path + "/Info";
        values.put(info + "/Type", TelemetryValue.constant("Motor"));
        values.put(info + "/Kind", TelemetryValue.constant(motor.kind().key()));
        values.put(info + "/Id", TelemetryValue.constant(motor.id()));
        values.put(info + "/Bus", TelemetryValue.constant(motor.canbus()));
        values.put(info + "/DeclaredNeutralMode", TelemetryValue.string(() -> motor.neutralMode().name()));
        values.put(info + "/DeclaredInverted", TelemetryValue.bool(motor::isInverted));
        values.put(info + "/Follower", TelemetryValue.constant(motor.follower() == null
                ? "" : motor.follower().leader().defaultName()));
        values.put(config + "/Disabled", overrides.motorDisabled(motor));
        values.put(config + "/NeutralMode", TelemetryValue.writableString(
                () -> overrides.config(motor).neutralMode().name(), value -> overrides.neutralMode(motor, value)));
        values.put(config + "/Inverted", TelemetryValue.writableBoolean(
                () -> overrides.config(motor).inverted(), value -> overrides.inverted(motor, value)));
        values.put(config + "/SupplyCurrentLimit", TelemetryValue.writableNumber(
                () -> overrides.config(motor).supplyCurrentLimitAmps(), value -> overrides.supplyLimit(motor, value)));
        values.put(config + "/StatorCurrentLimit", TelemetryValue.writableNumber(
                () -> overrides.config(motor).statorCurrentLimitAmps(), value -> overrides.statorLimit(motor, value)));
        values.put(config + "/Supported", TelemetryValue.bool(() -> overrides.supportsMotorConfig(motor)));
        values.put(config + "/Restore", TelemetryValue.command(() -> overrides.restore(motor)));
        values.put(config + "/Status", TelemetryValue.string(() -> overrides.motorStatus(motor)));
        values.put(state + "/PositionRotations", safeNumber(motor::positionRotations));
        values.put(state + "/VelocityRotationsPerSecond", safeNumber(motor::velocityRotationsPerSecond));
        values.put(state + "/AppliedVoltage", safeNumber(motor::appliedVoltage));
        values.put(state + "/SupplyCurrentAmps", safeNumber(motor::supplyCurrentAmps));
        values.put(state + "/StatorCurrentAmps", safeNumber(motor::statorCurrentAmps));
        values.put(state + "/CommandMode", TelemetryValue.string(() -> motor.command().mode().name()));
        values.put(state + "/CommandValue", TelemetryValue.number(() -> motor.command().value()));
    }

    private static void addEncoderTelemetry(
            String path,
            EncoderDevice encoder,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides) {
        values.put(path + "/Info/Type", TelemetryValue.constant("Encoder"));
        values.put(path + "/Info/Kind", TelemetryValue.constant(encoder.kind().key()));
        values.put(path + "/Info/Id", TelemetryValue.constant(encoder.id()));
        values.put(path + "/Info/Bus", TelemetryValue.constant(encoder.bus()));
        values.put(path + "/Info/DeclaredInverted", TelemetryValue.bool(encoder::isInverted));
        values.put(path + "/Info/GearRatio", TelemetryValue.number(encoder::gearRatio));
        values.put(path + "/Info/Conversion", TelemetryValue.number(encoder::conversion));
        values.put(path + "/Info/Offset", TelemetryValue.number(encoder::offset));
        values.put(path + "/Info/Units", TelemetryValue.string(() -> encoder.units().name()));
        values.put(path + "/State/Position", safeNumber(encoder::position));
        values.put(path + "/State/AbsolutePosition", safeNumber(encoder::absolutePosition));
        values.put(path + "/State/Velocity", safeNumber(encoder::velocity));
        values.put(path + "/Setup/RequestedPosition", TelemetryValue.writableNumber(
                () -> overrides.requestedEncoderPosition(encoder),
                value -> overrides.requestedEncoderPosition(encoder, value)));
        values.put(path + "/Setup/SetPosition", TelemetryValue.command(() ->
                overrides.encoderPosition(encoder, overrides.requestedEncoderPosition(encoder))));
        values.put(path + "/Setup/Zero", TelemetryValue.command(() -> overrides.encoderPosition(encoder, 0.0)));
        values.put(path + "/Setup/Supported", TelemetryValue.bool(() -> overrides.supportsEncoderPosition(encoder)));
        values.put(path + "/Setup/Status", TelemetryValue.string(() -> overrides.setupStatus(encoder)));
    }

    private static void addImuTelemetry(
            String path,
            ImuDevice imu,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides) {
        values.put(path + "/Info/Type", TelemetryValue.constant("IMU"));
        values.put(path + "/Info/Kind", TelemetryValue.constant(imu.kind().key()));
        values.put(path + "/Info/Id", TelemetryValue.constant(imu.id()));
        values.put(path + "/Info/Bus", TelemetryValue.constant(imu.canbus()));
        values.put(path + "/State/YawDegrees", safeNumber(imu::yawDegrees));
        values.put(path + "/State/AngleDegrees", safeNumber(imu::angleDegrees));
        values.put(path + "/State/PitchDegrees", safeNumber(imu::pitchDegrees));
        values.put(path + "/State/RollDegrees", safeNumber(imu::rollDegrees));
        values.put(path + "/State/YawRateDegreesPerSecond", safeNumber(imu::yawRateDegreesPerSecond));
        values.put(path + "/State/LinearAccelerationXG", safeNumber(imu::linearAccelerationXG));
        values.put(path + "/State/LinearAccelerationYG", safeNumber(imu::linearAccelerationYG));
        values.put(path + "/State/LinearAccelerationZG", safeNumber(imu::linearAccelerationZG));
        values.put(path + "/Setup/RequestedYawDegrees", TelemetryValue.writableNumber(
                () -> overrides.requestedImuYaw(imu), value -> overrides.requestedImuYaw(imu, value)));
        values.put(path + "/Setup/SetYaw", TelemetryValue.command(() ->
                overrides.imuYaw(imu, overrides.requestedImuYaw(imu))));
        values.put(path + "/Setup/ZeroYaw", TelemetryValue.command(() -> overrides.imuYaw(imu, 0.0)));
        values.put(path + "/Setup/Status", TelemetryValue.string(() -> overrides.setupStatus(imu)));
    }

    private static TelemetryValue safeNumber(DoubleSupplier reader) {
        return TelemetryValue.number(() -> {
            try {
                return reader.getAsDouble();
            } catch (RuntimeException exception) {
                return Double.NaN;
            }
        });
    }

    private static final class MutableTelemetryNode {
        private final String name;
        private final TelemetryNode.Kind kind;
        private final Map<String, MutableTelemetryNode> children = new LinkedHashMap<>();
        private final Map<String, TelemetryValue> values = new LinkedHashMap<>();
        private final Map<String, TelemetryAction> actions = new LinkedHashMap<>();

        private MutableTelemetryNode(String name, TelemetryNode.Kind kind) {
            this.name = name;
            this.kind = kind;
        }

        private MutableTelemetryNode child(String name, TelemetryNode.Kind kind) {
            return children.computeIfAbsent(name, ignored -> new MutableTelemetryNode(name, kind));
        }

        private void value(String path, TelemetryValue value) {
            String[] parts = path.split("/");
            MutableTelemetryNode node = this;
            for (int index = 0; index < parts.length - 1; index++) {
                if (!parts[index].isBlank()) node = node.child(parts[index], TelemetryNode.Kind.GROUP);
            }
            String leaf = parts[parts.length - 1];
            if (node.values.putIfAbsent(leaf, value) != null) {
                throw new IllegalArgumentException("Duplicate telemetry path '" + path + "'.");
            }
        }

        private TelemetryNode freeze() {
            Map<String, TelemetryNode> frozenChildren = new LinkedHashMap<>();
            children.forEach((childName, child) -> frozenChildren.put(childName, child.freeze()));
            return new TelemetryNode(name, kind, frozenChildren, values, actions);
        }
    }

    Map<String, HookBinding> hooks(Mechanism mechanism) {
        Map<String, HookBinding> hooks = new LinkedHashMap<>();
        collectHooks("", mechanism, Collections.newSetFromMap(new IdentityHashMap<>()), hooks);
        return hooks;
    }

    private void collectHooks(
            String path,
            Mechanism mechanism,
            Set<Mechanism> visited,
            Map<String, HookBinding> hooks) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = node(mechanism);
        node.hooks().forEach((name, hook) -> hooks.put(path + name, hook));
        node.children().forEach((name, child) -> collectHooks(path + name + ".", child, visited, hooks));
    }

    private void collectDeclarations(Mechanism mechanism, Set<Mechanism> visited, Set<Object> declarations) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = node(mechanism);
        for (Object declaration : node.declarations().values()) {
            collectDeclaration(declaration, declarations);
        }
        for (HookBinding hook : node.hooks().values()) {
            declarations.addAll(hook.event().declarations());
        }
        for (Mechanism child : node.children().values()) {
            collectDeclarations(child, visited, declarations);
        }
    }

    private boolean contains(Mechanism mechanism, Mechanism target, Set<Mechanism> visited) {
        if (mechanism == target) {
            return true;
        }
        if (!visited.add(mechanism)) {
            return false;
        }
        for (Mechanism child : node(mechanism).children().values()) {
            if (contains(child, target, visited)) {
                return true;
            }
        }
        return false;
    }

    private static void collectControlDeclarations(ControlBinding control, Set<Object> declarations) {
        declarations.addAll(control.motors());
        if (control.feedback() != null) {
            declarations.addAll(control.feedback().dependencies());
        }
        if (control.sink() != null) {
            declarations.addAll(control.sink().dependencies());
        }
        declarations.addAll(control.dependencies());
        for (ControlLoop loop : control.loops()) {
            declarations.addAll(loop.dependencies());
        }
    }

    private static void collectDeclaration(Object declaration, Set<Object> declarations) {
        if (declaration == null) {
            return;
        }
        if (declaration instanceof Iterable<?> values) {
            for (Object value : values) {
                collectDeclaration(value, declarations);
            }
            return;
        }
        declarations.add(declaration);
        if (declaration instanceof ControlBinding control) {
            collectControlDeclarations(control, declarations);
        } else if (declaration instanceof SimModel simulation) {
            collectSimDeclarations(simulation, declarations);
        }
    }

    private static void collectSimDeclarations(SimModel simulation, Set<Object> declarations) {
        declarations.addAll(simulation.motors());
        declarations.addAll(simulation.encoders());
        declarations.addAll(simulation.digitalInputs());
        for (MotorDevice motor : simulation.motors()) {
            declarations.add(motor.encoder());
        }
        if (simulation.range() != null) {
            declarations.add(simulation.range());
        }
        simulation.limits().forEach(limit -> {
            declarations.add(limit);
            declarations.add(limit.sensor());
        });
    }
}
