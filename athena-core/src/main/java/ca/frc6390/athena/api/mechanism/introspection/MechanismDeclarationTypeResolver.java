package ca.frc6390.athena.api.mechanism.introspection;

import java.util.Objects;

import ca.frc6390.athena.api.mechanism.MechanismConfig;
import ca.frc6390.athena.api.mechanism.StatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;

public final class MechanismDeclarationTypeResolver {
    private MechanismDeclarationTypeResolver() {
    }

    public static Class<?> resolve(Object declaration) {
        Objects.requireNonNull(declaration, "declaration");
        return resolve(declaration.getClass());
    }

    public static Class<?> resolve(Class<?> runtimeType) {
        Objects.requireNonNull(runtimeType, "runtimeType");
        for (Class<?> iface : runtimeType.getInterfaces()) {
            Class<?> resolved = resolveInterface(iface);
            if (resolved != null) {
                return resolved;
            }
        }
        Class<?> superType = runtimeType.getSuperclass();
        if (superType != null && superType != Object.class) {
            if (MechanismConfig.class.isAssignableFrom(superType) && !isFrameworkType(superType)) {
                return superType;
            }
            return resolve(superType);
        }
        return runtimeType;
    }

    private static Class<?> resolveInterface(Class<?> iface) {
        if (!MechanismConfig.class.isAssignableFrom(iface) || isFrameworkType(iface)) {
            return null;
        }
        return iface;
    }

    private static boolean isFrameworkType(Class<?> type) {
        return type == MechanismConfig.class
                || type == StatefulMechanismConfig.class
                || type == TypedStatefulMechanismConfig.class;
    }
}
