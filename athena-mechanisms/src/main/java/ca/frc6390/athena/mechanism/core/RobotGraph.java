package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimLimit;
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
            }
        }
        return simulations;
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
        declarations.addAll(control.feedback());
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
        for (MotorDevice motor : simulation.motors()) {
            declarations.add(motor.encoder());
        }
        for (Object declaration : simulation.dependencies()) {
            declarations.add(declaration);
            if (declaration instanceof ControlBinding control) {
                collectControlDeclarations(control, declarations);
            } else if (declaration instanceof SimLimit limit) {
                declarations.add(limit.sensor());
            }
        }
    }
}
