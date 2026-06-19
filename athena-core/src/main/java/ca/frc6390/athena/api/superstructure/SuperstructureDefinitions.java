package ca.frc6390.athena.api.superstructure;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

import ca.frc6390.athena.api.ConfigDeclarations;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;
import ca.frc6390.athena.api.superstructure.runtime.SuperstructureRuntimeLowerer;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureRuntimeConfig;

public final class SuperstructureDefinitions {
    private SuperstructureDefinitions() {
    }

    public static <S, SP> SuperstructureDefinition<SP> structured(SuperstructureConfig<S, SP> declaration) {
        FlowSuperstructureConfig<S, SP> builder = new FlowSuperstructureConfig<>(declaration.name(), declaration.initialState());
        declaration.mechanisms(builder.mechanismsSection());
        declaration.inputs(builder.inputsSection());
        declaration.behavior(builder.behaviorSection());
        return builder.definition();
    }

    @SuppressWarnings("unchecked")
    public static <S, SP> SuperstructureDefinition<SP> structured(Class<?> declarationType) {
        Method configure = configureMethod(declarationType);
        if (configure != null) {
            SuperstructureSpec<S, SP> spec = new SuperstructureSpec<>();
            invokeConfigure(configure, spec);
            return spec.definition();
        }
        if (SuperstructureConfig.class.isAssignableFrom(declarationType)) {
            return structured((SuperstructureConfig<S, SP>) ConfigDeclarations.instance(
                declarationType.asSubclass(SuperstructureConfig.class)));
        }
        throw new IllegalArgumentException(
            "Superstructure declaration needs static configure(SuperstructureSpec) or must implement SuperstructureConfig: "
                + declarationType.getName());
    }

    public static <SP> SuperstructureRuntimeConfig<?, SP> runtime(SuperstructureDefinition<SP> definition) {
        return SuperstructureRuntimeLowerer.lower(definition);
    }

    public static <SP> SuperstructureMechanism<?, SP> build(SuperstructureDefinition<SP> definition) {
        return SuperstructureRuntimeLowerer.build(definition);
    }

    public static <S, SP> SuperstructureMechanism<?, SP> build(
            Class<?> declarationType) {
        return build(structured(declarationType));
    }

    private static Method configureMethod(Class<?> declarationType) {
        if (declarationType == null) {
            throw new NullPointerException("declarationType");
        }
        try {
            Method method = declarationType.getDeclaredMethod("configure", SuperstructureSpec.class);
            if (!Modifier.isStatic(method.getModifiers())) {
                throw new IllegalArgumentException(
                    "configure(SuperstructureSpec) must be static: " + declarationType.getName());
            }
            method.setAccessible(true);
            return method;
        } catch (NoSuchMethodException ignored) {
            return null;
        }
    }

    private static void invokeConfigure(Method method, SuperstructureSpec<?, ?> spec) {
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
