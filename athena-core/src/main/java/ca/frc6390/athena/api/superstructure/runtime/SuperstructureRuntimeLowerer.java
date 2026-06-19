package ca.frc6390.athena.api.superstructure.runtime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.Function;
import java.util.function.IntSupplier;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.superstructure.SuperstructureDefinitions;
import ca.frc6390.athena.api.superstructure.definition.ExistingMechanismChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.ExistingNestedSuperstructureChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.MechanismChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.NestedSuperstructureChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureBooleanInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureChildDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureConstraintDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDoubleInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureHookDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureHookPhase;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureIntInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureObjectInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructurePose2dInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructurePose3dInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureStringInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureTransitionHookDefinition;
import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureRuntimeConfig;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public final class SuperstructureRuntimeLowerer {
    private SuperstructureRuntimeLowerer() {
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    public static <SP> SuperstructureRuntimeConfig<?, SP> lower(SuperstructureDefinition<SP> definition) {
        Object initialState = definition.initialState()
            .orElseThrow(() -> new IllegalArgumentException(
                "superstructure build requires an initial state object"));
        List<SuperstructureMechanism.Child<SP, ?>> children = new ArrayList<>();
        for (SuperstructureChildDefinition<SP> child : definition.children()) {
            SuperstructureMechanism.Child<SP, ?> built = buildChild(child);
            if (built != null) {
                children.add(built);
            }
        }

        Map<String, BooleanSupplier> boolInputs = new LinkedHashMap<>();
        Map<String, DoubleSupplier> doubleInputs = new LinkedHashMap<>();
        Map<String, IntSupplier> intInputs = new LinkedHashMap<>();
        Map<String, Supplier<String>> stringInputs = new LinkedHashMap<>();
        Map<String, Supplier<Pose2d>> pose2dInputs = new LinkedHashMap<>();
        Map<String, Supplier<Pose3d>> pose3dInputs = new LinkedHashMap<>();
        Map<String, Supplier<?>> objectInputs = new LinkedHashMap<>();
        addInputs(definition.inputs(), boolInputs, doubleInputs, intInputs, stringInputs, pose2dInputs, pose3dInputs, objectInputs);

        @SuppressWarnings("rawtypes")
        Map constraints = new LinkedHashMap();
        addConstraints(definition, initialState, constraints);

        @SuppressWarnings("rawtypes")
        Map enterBindings = new LinkedHashMap();
        @SuppressWarnings("rawtypes")
        List transitionBindings = new ArrayList();
        @SuppressWarnings("rawtypes")
        Map bindings = new LinkedHashMap();
        List<SuperstructureRuntimeConfig.Binding<SP>> alwaysBindings = new ArrayList<>();
        List<SuperstructureRuntimeConfig.Binding<SP>> periodicBindings = new ArrayList<>();
        @SuppressWarnings("rawtypes")
        Map exitBindings = new LinkedHashMap();
        List<SuperstructureRuntimeConfig.Binding<SP>> exitAlwaysBindings = new ArrayList<>();
        @SuppressWarnings("rawtypes")
        Map lifecycleBindings = new LinkedHashMap();
        @SuppressWarnings("rawtypes")
        List lifecycleExitBindings = new ArrayList();

        addHooks(
            definition,
            initialState,
            enterBindings,
            transitionBindings,
            bindings,
            alwaysBindings,
            periodicBindings,
            exitBindings,
            exitAlwaysBindings,
            lifecycleBindings,
            lifecycleExitBindings);

        return new SuperstructureRuntimeConfig<>(
            initialState,
            Math.max(0.0, definition.stateMachineDelaySeconds()),
            children,
            constraints,
            List.of(),
            boolInputs,
            doubleInputs,
            intInputs,
            stringInputs,
            pose2dInputs,
            pose3dInputs,
            objectInputs,
            enterBindings,
            transitionBindings,
            bindings,
            alwaysBindings,
            periodicBindings,
            exitBindings,
            exitAlwaysBindings,
            lifecycleBindings,
            lifecycleExitBindings);
    }

    public static <SP> SuperstructureMechanism<?, SP> build(SuperstructureDefinition<SP> definition) {
        return SuperstructureMechanism.create(lower(definition), definition);
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <SP> SuperstructureMechanism.Child<SP, ?> buildChild(SuperstructureChildDefinition<SP> child) {
        if (child instanceof MechanismChildDefinition mechanismChild) {
            Mechanism mechanism = MechanismDefinitions.build(mechanismChild.mechanismDefinition());
            if (mechanism == null) {
                return null;
            }
            Class<?> stateType = mechanismChild.mechanismDefinition()
                .initialState()
                .map(Object::getClass)
                .or(() -> mechanismChild.mechanismDefinition().stateType())
                .orElse(null);
            return new SuperstructureMechanism.Child(mechanism, (Function) mechanismChild.mapper(), stateType);
        }
        if (child instanceof ExistingMechanismChildDefinition existingMechanism) {
            Object goal = ((StatefulMechanism<?>) existingMechanism.mechanism()).stateMachine().goal();
            Class<?> stateType = goal != null ? goal.getClass() : null;
            return new SuperstructureMechanism.Child((StatefulMechanism) existingMechanism.mechanism(),
                (Function) existingMechanism.mapper(), stateType);
        }
        if (child instanceof NestedSuperstructureChildDefinition<?, ?, ?> nested) {
            SuperstructureMechanism<?, ?> superstructure = SuperstructureDefinitions.build(nested.superstructureDefinition());
            if (superstructure == null) {
                return null;
            }
            Class<?> stateType = nested.superstructureDefinition().initialState().isPresent()
                ? nested.superstructureDefinition().initialState().get().getClass()
                : nested.superstructureDefinition().stateType();
            return new SuperstructureMechanism.Child((SuperstructureMechanism) superstructure,
                (Function) nested.mapper(), stateType);
        }
        if (child instanceof ExistingNestedSuperstructureChildDefinition existingNested) {
            Object goal = existingNested.superstructure().stateMachine().goal();
            Class<?> stateType = goal != null ? goal.getClass() : null;
            return new SuperstructureMechanism.Child((SuperstructureMechanism) existingNested.superstructure(),
                (Function) existingNested.mapper(), stateType);
        }
        throw new IllegalArgumentException("Unsupported V2 superstructure child: " + child.getClass().getName());
    }

    @SuppressWarnings("unchecked")
    private static <SP> void addInputs(
            List<SuperstructureInputDefinition> definitions,
            Map<String, BooleanSupplier> boolInputs,
            Map<String, DoubleSupplier> doubleInputs,
            Map<String, IntSupplier> intInputs,
            Map<String, Supplier<String>> stringInputs,
            Map<String, Supplier<Pose2d>> pose2dInputs,
            Map<String, Supplier<Pose3d>> pose3dInputs,
            Map<String, Supplier<?>> objectInputs) {
        for (SuperstructureInputDefinition input : definitions) {
            if (input instanceof SuperstructureBooleanInputDefinition boolInput) {
                boolInputs.put(boolInput.name(), boolInput.supplier());
            } else if (input instanceof SuperstructureDoubleInputDefinition doubleInput) {
                doubleInputs.put(doubleInput.name(), doubleInput.supplier());
            } else if (input instanceof SuperstructureIntInputDefinition intInput) {
                intInputs.put(intInput.name(), intInput.supplier());
            } else if (input instanceof SuperstructureStringInputDefinition stringInput) {
                stringInputs.put(stringInput.name(), stringInput.supplier());
            } else if (input instanceof SuperstructurePose2dInputDefinition pose2dInput) {
                pose2dInputs.put(pose2dInput.name(), pose2dInput.supplier());
            } else if (input instanceof SuperstructurePose3dInputDefinition pose3dInput) {
                pose3dInputs.put(pose3dInput.name(), pose3dInput.supplier());
            } else if (input instanceof SuperstructureObjectInputDefinition objectInput) {
                objectInputs.put(objectInput.name(), objectInput.supplier());
            }
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <SP> void addConstraints(
            SuperstructureDefinition<SP> definition,
            Object initialState,
            @SuppressWarnings("rawtypes") Map constraints) {
        for (SuperstructureConstraintDefinition<SP> constraint : definition.constraints()) {
            Object state = resolveState(initialState, constraint.state());
            List<Object> transitions = new ArrayList<>();
            for (String transition : constraint.transitionStates()) {
                transitions.add(resolveState(initialState, transition));
            }
            constraints.put(state, new SuperstructureRuntimeConfig.Constraint<>(constraint.guard(), transitions));
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <SP> void addHooks(
            SuperstructureDefinition<SP> definition,
            Object initialState,
            @SuppressWarnings("rawtypes") Map enterBindings,
            @SuppressWarnings("rawtypes") List transitionBindings,
            @SuppressWarnings("rawtypes") Map bindings,
            List<SuperstructureRuntimeConfig.Binding<SP>> alwaysBindings,
            List<SuperstructureRuntimeConfig.Binding<SP>> periodicBindings,
            @SuppressWarnings("rawtypes") Map exitBindings,
            List<SuperstructureRuntimeConfig.Binding<SP>> exitAlwaysBindings,
            @SuppressWarnings("rawtypes") Map lifecycleBindings,
            @SuppressWarnings("rawtypes") List lifecycleExitBindings) {
        for (SuperstructureHookDefinition<SP> hook : definition.hooks()) {
            List<Object> states = resolveStates(initialState, hook.states());
            switch (hook.phase()) {
                case STATE_ENTER -> addStateBindings(enterBindings, states, hook.callback()::apply);
                case STATE_PERIODIC -> addStateBindings(bindings, states, hook.callback()::apply);
                case STATE_EXIT -> addStateBindings(exitBindings, states, hook.callback()::apply);
                case ANY_STATE_EXIT -> exitAlwaysBindings.add(hook.callback()::apply);
                case ALWAYS -> alwaysBindings.add(hook.callback()::apply);
                case ROBOT_PERIODIC -> {
                    if (states.isEmpty()) {
                        periodicBindings.add(hook.callback()::apply);
                    } else {
                        ((List) lifecycleBindings.computeIfAbsent(RobotCoreHooks.Phase.ROBOT_PERIODIC, unused -> new ArrayList<>()))
                            .add(new SuperstructureRuntimeConfig.LifecycleHookBinding<>(hook.callback()::apply, states));
                    }
                }
                case ROBOT_PHASE -> {
                    RobotCoreHooks.Phase phase = hook.robotPhase().orElseThrow();
                    ((List) lifecycleBindings.computeIfAbsent(phase, unused -> new ArrayList<>()))
                        .add(new SuperstructureRuntimeConfig.LifecycleHookBinding<>(hook.callback()::apply, states));
                }
                case ROBOT_PHASE_EXIT ->
                    lifecycleExitBindings.add(new SuperstructureRuntimeConfig.LifecycleHookBinding<>(hook.callback()::apply, states));
            }
        }

        for (SuperstructureTransitionHookDefinition<SP> transition : definition.transitionHooks()) {
            Object from = resolveState(initialState, transition.fromState());
            Object to = resolveState(initialState, transition.toState());
            transitionBindings.add(new SuperstructureRuntimeConfig.TransitionBinding(
                from,
                to,
                (context, resolvedFrom, resolvedTo) -> transition.callback().apply(context, resolvedFrom, resolvedTo)));
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <SP> void addStateBindings(
            Map target,
            List<Object> states,
            SuperstructureRuntimeConfig.Binding<SP> binding) {
        for (Object state : states) {
            ((List<SuperstructureRuntimeConfig.Binding<SP>>) target.computeIfAbsent(state, unused -> new ArrayList<>()))
                .add(binding);
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static List<Object> resolveStates(Object initialState, List<String> stateNames) {
        if (stateNames == null || stateNames.isEmpty()) {
            return List.of();
        }
        List<Object> states = new ArrayList<>(stateNames.size());
        for (String stateName : stateNames) {
            states.add(resolveState(initialState, stateName));
        }
        return List.copyOf(states);
    }

    private static Object resolveState(Object context, String stateName) {
        Object state = StateSpecAccess.resolve(context, stateName);
        if (state == null) {
            throw new IllegalArgumentException("Unknown superstructure state '" + stateName + "'");
        }
        return state;
    }
}
