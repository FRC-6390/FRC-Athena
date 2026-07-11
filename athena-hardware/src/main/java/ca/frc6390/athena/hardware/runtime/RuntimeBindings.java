package ca.frc6390.athena.hardware.runtime;

import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Objects;

/** Identity-based runtime bindings used by directly readable declarations. */
public final class RuntimeBindings<K, V> {
    private final Map<K, Map<RuntimeScope, V>> bindings = new IdentityHashMap<>();

    public AutoCloseable bind(K declaration, RuntimeScope scope, V value) {
        Objects.requireNonNull(declaration, "declaration");
        Objects.requireNonNull(scope, "scope");
        Objects.requireNonNull(value, "value");
        synchronized (bindings) {
            bindings.computeIfAbsent(declaration, ignored -> new IdentityHashMap<>()).put(scope, value);
        }
        return () -> unbind(declaration, scope, value);
    }

    public V get(K declaration, String description) {
        Objects.requireNonNull(declaration, "declaration");
        Map<RuntimeScope, V> values;
        synchronized (bindings) {
            Map<RuntimeScope, V> bound = bindings.get(declaration);
            values = bound == null ? Map.of() : new IdentityHashMap<>(bound);
        }
        if (values.isEmpty()) {
            throw new IllegalStateException(description + " is not runtime-bound.");
        }
        if (values.size() == 1) {
            return values.values().iterator().next();
        }
        RuntimeScope current = RuntimeScope.current();
        V selected = current == null ? null : values.get(current);
        if (selected == null) {
            throw new IllegalStateException(
                    description + " is bound to multiple runtimes; read it inside the owning runtime scope.");
        }
        return selected;
    }

    public V find(K declaration) {
        Objects.requireNonNull(declaration, "declaration");
        synchronized (bindings) {
            Map<RuntimeScope, V> values = bindings.get(declaration);
            if (values == null || values.isEmpty()) {
                return null;
            }
            if (values.size() == 1) {
                return values.values().iterator().next();
            }
            RuntimeScope current = RuntimeScope.current();
            V selected = current == null ? null : values.get(current);
            if (selected == null) {
                throw new IllegalStateException(
                        "Declaration is bound to multiple runtimes; read it inside the owning runtime scope.");
            }
            return selected;
        }
    }

    private void unbind(K declaration, RuntimeScope scope, V value) {
        synchronized (bindings) {
            Map<RuntimeScope, V> values = bindings.get(declaration);
            if (values == null || values.get(scope) != value) {
                return;
            }
            values.remove(scope);
            if (values.isEmpty()) {
                bindings.remove(declaration);
            }
        }
    }
}
