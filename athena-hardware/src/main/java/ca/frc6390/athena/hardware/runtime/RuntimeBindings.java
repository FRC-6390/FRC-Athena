package ca.frc6390.athena.hardware.runtime;

import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Objects;

/** Identity-based runtime bindings used by directly readable declarations. */
public final class RuntimeBindings<K, V> {
    private final Object mutationLock = new Object();
    private volatile Map<K, BindingGroup<V>> bindings = Map.of();

    public AutoCloseable bind(K declaration, RuntimeScope scope, V value) {
        Objects.requireNonNull(declaration, "declaration");
        Objects.requireNonNull(scope, "scope");
        Objects.requireNonNull(value, "value");
        Binding<V> binding = new Binding<>(value);
        synchronized (mutationLock) {
            IdentityHashMap<K, BindingGroup<V>> updated = new IdentityHashMap<>(bindings);
            IdentityHashMap<RuntimeScope, Binding<V>> values = new IdentityHashMap<>();
            BindingGroup<V> current = updated.get(declaration);
            if (current != null) {
                values.putAll(current.values);
            }
            values.put(scope, binding);
            updated.put(declaration, new BindingGroup<>(values));
            bindings = immutableIdentityMap(updated);
        }
        return () -> unbind(declaration, scope, binding);
    }

    public V get(K declaration, String description) {
        Objects.requireNonNull(declaration, "declaration");
        BindingGroup<V> group = bindings.get(declaration);
        if (group == null) {
            throw new IllegalStateException(description + " is not runtime-bound.");
        }
        if (group.single != null) {
            return group.single.value;
        }
        RuntimeScope current = RuntimeScope.current();
        Binding<V> selected = current == null ? null : group.values.get(current);
        if (selected == null) {
            throw new IllegalStateException(
                    description + " is bound to multiple runtimes; read it inside the owning runtime scope.");
        }
        return selected.value;
    }

    public V find(K declaration) {
        Objects.requireNonNull(declaration, "declaration");
        BindingGroup<V> group = bindings.get(declaration);
        if (group == null) {
            return null;
        }
        if (group.single != null) {
            return group.single.value;
        }
        RuntimeScope current = RuntimeScope.current();
        Binding<V> selected = current == null ? null : group.values.get(current);
        if (selected == null) {
            throw new IllegalStateException(
                    "Declaration is bound to multiple runtimes; read it inside the owning runtime scope.");
        }
        return selected.value;
    }

    private void unbind(K declaration, RuntimeScope scope, Binding<V> binding) {
        synchronized (mutationLock) {
            BindingGroup<V> current = bindings.get(declaration);
            if (current == null || current.values.get(scope) != binding) {
                return;
            }

            IdentityHashMap<K, BindingGroup<V>> updated = new IdentityHashMap<>(bindings);
            IdentityHashMap<RuntimeScope, Binding<V>> values = new IdentityHashMap<>(current.values);
            values.remove(scope);
            if (values.isEmpty()) {
                updated.remove(declaration);
            } else {
                updated.put(declaration, new BindingGroup<>(values));
            }
            bindings = immutableIdentityMap(updated);
        }
    }

    private static <K, V> Map<K, V> immutableIdentityMap(Map<? extends K, ? extends V> values) {
        return Collections.unmodifiableMap(new IdentityHashMap<>(values));
    }

    private static final class Binding<V> {
        private final V value;

        private Binding(V value) {
            this.value = value;
        }
    }

    private static final class BindingGroup<V> {
        private final Map<RuntimeScope, Binding<V>> values;
        private final Binding<V> single;

        private BindingGroup(Map<RuntimeScope, Binding<V>> values) {
            this.values = immutableIdentityMap(values);
            this.single = values.size() == 1 ? values.values().iterator().next() : null;
        }
    }
}
