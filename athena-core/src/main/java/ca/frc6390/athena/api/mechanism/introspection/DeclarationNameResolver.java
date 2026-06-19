package ca.frc6390.athena.api.mechanism.introspection;

import java.lang.annotation.Annotation;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Objects;
import java.util.Optional;

public final class DeclarationNameResolver {
    private DeclarationNameResolver() {
    }

    public static String publicName(java.lang.reflect.Field field, Annotation annotation) {
        Objects.requireNonNull(field, "field");
        Objects.requireNonNull(annotation, "annotation");
        return explicitName(annotation).orElse(field.getName());
    }

    public static String publicName(Method method, Annotation annotation) {
        Objects.requireNonNull(method, "method");
        Objects.requireNonNull(annotation, "annotation");
        return explicitName(annotation).orElse(method.getName());
    }

    public static Optional<String> explicitName(Annotation annotation) {
        Objects.requireNonNull(annotation, "annotation");
        return readStringMember(annotation, "value")
            .or(() -> readStringMember(annotation, "name"));
    }

    private static Optional<String> readStringMember(Annotation annotation, String memberName) {
        try {
            Method member = annotation.annotationType().getMethod(memberName);
            if (!String.class.equals(member.getReturnType())) {
                return Optional.empty();
            }
            Object value = member.invoke(annotation);
            if (!(value instanceof String stringValue)) {
                return Optional.empty();
            }
            String trimmed = stringValue.trim();
            return trimmed.isEmpty() ? Optional.empty() : Optional.of(trimmed);
        } catch (NoSuchMethodException ignored) {
            return Optional.empty();
        } catch (IllegalAccessException | InvocationTargetException ex) {
            throw new IllegalStateException("Failed to read annotation member '" + memberName + "'", ex);
        }
    }
}
