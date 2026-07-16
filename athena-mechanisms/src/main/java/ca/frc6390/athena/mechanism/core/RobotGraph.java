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

    Map<String, TelemetryValue> telemetry(
            Collection<Mechanism> mechanisms, RuntimeOverrides overrides) {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Mechanism mechanism : mechanisms) {
            collectTelemetry(node(mechanism).name(), mechanism, visited, values, overrides);
        }
        return Collections.unmodifiableMap(values);
    }

    private void collectTelemetry(
            String path,
            Mechanism mechanism,
            Set<Mechanism> visited,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides) {
        if (!visited.add(mechanism)) return;
        MechanismNode node = node(mechanism);
        node.declarations().forEach((name, declaration) ->
                addTelemetry(path, name, declaration, values, overrides));
        node.children().forEach((name, child) ->
                collectTelemetry(path + "/" + name, child, visited, values, overrides));
    }

    private static void addTelemetry(
            String mechanismPath,
            String valuePath,
            Object declaration,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides) {
        if (declaration instanceof TelemetryValue value) {
            values.put(mechanismPath + "/" + valuePath, value);
        } else if (declaration instanceof MotorDevice motor) {
            addMotorTelemetry(mechanismPath + "/" + valuePath, motor, values, overrides);
        } else if (declaration instanceof EncoderDevice encoder) {
            addEncoderTelemetry(mechanismPath + "/" + valuePath, encoder, values);
        } else if (declaration instanceof ImuDevice imu) {
            addImuTelemetry(mechanismPath + "/" + valuePath, imu, values);
        } else if (declaration instanceof DigitalInputDevice input) {
            values.put(mechanismPath + "/" + valuePath + "/active", TelemetryValue.bool(input::active));
            values.put(mechanismPath + "/" + valuePath + "/raw", TelemetryValue.bool(input::raw));
        } else if (declaration instanceof ControlBinding control) {
            String controlPath = mechanismPath + "/" + valuePath;
            values.put(controlPath + "/disabled", overrides.controlDisabled(control));
            values.put(controlPath + "/mode", TelemetryValue.string(() -> control.mode().name()));
            values.put(controlPath + "/slot", TelemetryValue.number(() -> control.slot()));
            for (int index = 0; index < control.motors().size(); index++) {
                addMotorTelemetry(
                        controlPath + "/motors/" + index,
                        control.motors().get(index),
                        values,
                        overrides);
            }
            if (control.feedback() != null) {
                values.put(controlPath + "/feedback/position",
                        safeNumber(control.feedback().position()::position));
                values.put(controlPath + "/feedback/velocity",
                        safeNumber(control.feedback().velocity()::velocity));
            }
            int customLoop = 0;
            for (ControlLoop loop : control.loops()) {
                String loopName = loop instanceof ca.frc6390.athena.mechanism.control.PidGains ? "pid"
                        : loop instanceof ca.frc6390.athena.mechanism.control.FeedforwardGains ? "feedforward"
                        : "loop" + customLoop++;
                addTelemetry(mechanismPath, valuePath + "/" + loopName, loop, values, overrides);
            }
        } else if (declaration instanceof TelemetrySource source) {
            source.telemetry().forEach((name, value) -> addTelemetry(
                    mechanismPath, valuePath + "/" + name, value, values, overrides));
        } else if (declaration instanceof Iterable<?> nested) {
            int index = 0;
            for (Object value : nested) {
                addTelemetry(mechanismPath, valuePath + "/" + index++, value, values, overrides);
            }
        }
    }

    private static void addMotorTelemetry(
            String path,
            MotorDevice motor,
            Map<String, TelemetryValue> values,
            RuntimeOverrides overrides) {
        values.put(path + "/disabled", overrides.motorDisabled(motor));
        values.put(path + "/positionRotations", safeNumber(motor::positionRotations));
        values.put(path + "/velocityRotationsPerSecond", safeNumber(motor::velocityRotationsPerSecond));
        values.put(path + "/appliedVoltage", safeNumber(motor::appliedVoltage));
        values.put(path + "/supplyCurrentAmps", safeNumber(motor::supplyCurrentAmps));
        values.put(path + "/statorCurrentAmps", safeNumber(motor::statorCurrentAmps));
        values.put(path + "/command/mode", TelemetryValue.string(() -> motor.command().mode().name()));
        values.put(path + "/command/value", TelemetryValue.number(() -> motor.command().value()));
    }

    private static void addEncoderTelemetry(
            String path,
            EncoderDevice encoder,
            Map<String, TelemetryValue> values) {
        values.put(path + "/position", safeNumber(encoder::position));
        values.put(path + "/absolutePosition", safeNumber(encoder::absolutePosition));
        values.put(path + "/velocity", safeNumber(encoder::velocity));
    }

    private static void addImuTelemetry(
            String path,
            ImuDevice imu,
            Map<String, TelemetryValue> values) {
        values.put(path + "/yawDegrees", safeNumber(imu::yawDegrees));
        values.put(path + "/angleDegrees", safeNumber(imu::angleDegrees));
        values.put(path + "/pitchDegrees", safeNumber(imu::pitchDegrees));
        values.put(path + "/rollDegrees", safeNumber(imu::rollDegrees));
        values.put(path + "/yawRateDegreesPerSecond", safeNumber(imu::yawRateDegreesPerSecond));
        values.put(path + "/linearAccelerationXG", safeNumber(imu::linearAccelerationXG));
        values.put(path + "/linearAccelerationYG", safeNumber(imu::linearAccelerationYG));
        values.put(path + "/linearAccelerationZG", safeNumber(imu::linearAccelerationZG));
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
