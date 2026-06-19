package ca.frc6390.athena.api.mechanism;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Optional;

import ca.frc6390.athena.api.ConfigDeclarations;
import ca.frc6390.athena.api.mechanism.introspection.AnnotatedMechanismDefinitionLoader;
import ca.frc6390.athena.api.mechanism.runtime.MechanismRuntimeLowerer;
import ca.frc6390.athena.api.mechanism.validation.MechanismDefinitionValidator;

public final class MechanismDefinitions {
    private MechanismDefinitions() {
    }

    public static ca.frc6390.athena.api.mechanism.definition.MechanismDefinition annotation(Class<?> declarationType) {
        return AnnotatedMechanismDefinitionLoader.load(declarationType);
    }

    public static ca.frc6390.athena.api.mechanism.definition.MechanismDefinition annotation(Object declaration) {
        return AnnotatedMechanismDefinitionLoader.load(declaration);
    }

    public static ca.frc6390.athena.api.mechanism.definition.MechanismDefinition structured(MechanismConfig declaration) {
        ca.frc6390.athena.api.mechanism.definition.MechanismDefinition annotated =
            AnnotatedMechanismDefinitionLoader.load(declaration);
        FlowMechanismConfig builder = FlowMechanismConfig.fromDefinition(annotated);
        if (!declaration.name().isBlank()) {
            builder.named(declaration.name());
        }
        builder.disabled(declaration.disabled());
        declaration.identity(builder.identitySection());
        declaration.motors(builder.motorsSection());
        declaration.encoders(builder.encodersSection());
        declaration.inputs(builder.inputsSection());
        declaration.behavior(builder.behaviorSection());
        if (declaration instanceof StatefulMechanismConfig stateful) {
            Object initialState = stateful.initialState();
            builder.state(Optional.ofNullable(stateful.stateType()),
                    Optional.ofNullable(ca.frc6390.athena.mechanisms.statespec.StateNames.name(initialState)));
            builder.initialState(Optional.ofNullable(initialState));
            builder.stateMachineDelaySecondsInternal(stateful.stateMachineDelaySeconds());
        } else if (declaration instanceof TypedStatefulMechanismConfig<?> stateful) {
            Object initialState = stateful.initialState();
            builder.state(Optional.ofNullable(stateful.stateType()),
                    Optional.ofNullable(ca.frc6390.athena.mechanisms.statespec.StateNames.name(initialState)));
            builder.initialState(Optional.ofNullable(initialState));
            builder.stateMachineDelaySecondsInternal(stateful.stateMachineDelaySeconds());
        }
        return builder.definition();
    }

    public static ca.frc6390.athena.api.mechanism.definition.MechanismDefinition structured(
            Class<?> declarationType) {
        Method configure = configureMethod(declarationType);
        if (configure != null) {
            ca.frc6390.athena.api.mechanism.definition.MechanismDefinition annotated =
                AnnotatedMechanismDefinitionLoader.load(declarationType);
            MechanismSpec spec = new MechanismSpec(FlowMechanismConfig.fromDefinition(annotated));
            invokeConfigure(configure, spec);
            return spec.definition();
        }
        if (MechanismConfig.class.isAssignableFrom(declarationType)) {
            return structured(ConfigDeclarations.instance(declarationType.asSubclass(MechanismConfig.class)));
        }
        throw new IllegalArgumentException(
            "Mechanism declaration needs static configure(MechanismSpec) or must implement MechanismConfig: "
                + declarationType.getName());
    }

    public static java.util.List<ca.frc6390.athena.api.mechanism.validation.MechanismValidationIssue> validate(
            ca.frc6390.athena.api.mechanism.definition.MechanismDefinition definition) {
        return MechanismDefinitionValidator.validate(definition);
    }

    public static ca.frc6390.athena.mechanisms.Mechanism build(
            ca.frc6390.athena.api.mechanism.definition.MechanismDefinition definition) {
        return MechanismRuntimeLowerer.build(definition);
    }

    public static ca.frc6390.athena.mechanisms.Mechanism build(
            Class<?> declarationType) {
        return build(structured(declarationType));
    }

    private static Method configureMethod(Class<?> declarationType) {
        if (declarationType == null) {
            throw new NullPointerException("declarationType");
        }
        try {
            Method method = declarationType.getDeclaredMethod("configure", MechanismSpec.class);
            if (!Modifier.isStatic(method.getModifiers())) {
                throw new IllegalArgumentException(
                    "configure(MechanismSpec) must be static: " + declarationType.getName());
            }
            method.setAccessible(true);
            return method;
        } catch (NoSuchMethodException ignored) {
            return null;
        }
    }

    private static void invokeConfigure(Method method, MechanismSpec spec) {
        try {
            method.invoke(null, spec);
        } catch (IllegalAccessException ex) {
            throw new IllegalStateException("Failed to access " + method, ex);
        } catch (InvocationTargetException ex) {
            Throwable cause = ex.getCause();
            if (cause instanceof RuntimeException runtime) {
                throw runtime;
            }
            if (cause instanceof Error error) {
                throw error;
            }
            throw new IllegalStateException("Failed to run " + method, cause);
        }
    }
}
