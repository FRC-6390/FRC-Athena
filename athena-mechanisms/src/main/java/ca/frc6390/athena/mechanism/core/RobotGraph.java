package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimLimit;
import ca.frc6390.athena.hardware.sim.SimModel;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
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

    private void collectDeclarations(Mechanism mechanism, Set<Mechanism> visited, Set<Object> declarations) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = node(mechanism);
        for (Object declaration : node.declarations().values()) {
            declarations.add(declaration);
            if (declaration instanceof ControlBinding control) {
                collectControlDeclarations(control, declarations);
            } else if (declaration instanceof SimModel simulation) {
                collectSimDeclarations(simulation, declarations);
            }
        }
        for (Mechanism child : node.children().values()) {
            collectDeclarations(child, visited, declarations);
        }
    }

    private static void collectControlDeclarations(ControlBinding control, Set<Object> declarations) {
        declarations.addAll(control.motors());
        declarations.addAll(control.feedback());
        declarations.addAll(control.dependencies());
        for (ControlLoop loop : control.loops()) {
            declarations.addAll(loop.dependencies());
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
