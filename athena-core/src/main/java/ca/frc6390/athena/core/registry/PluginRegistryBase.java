package ca.frc6390.athena.core.registry;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;
import java.util.Set;
import java.util.function.Consumer;

/**
 * Shared base for keyed plugin registries backed by {@link ServiceLoader} providers.
 */
public abstract class PluginRegistryBase<V> {
    private final Map<String, V> values = new HashMap<>();

    protected final void put(String key, V value) {
        values.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(value, "value"));
    }

    protected final V require(String key) {
        return Objects.requireNonNull(values.get(key), missingMessage(key));
    }

    protected abstract String missingMessage(String key);

    protected static <P> void loadProviders(
            Class<P> providerType,
            Class<?> ownerType,
            Consumer<P> registrar) {
        Objects.requireNonNull(providerType, "providerType");
        Objects.requireNonNull(ownerType, "ownerType");
        Objects.requireNonNull(registrar, "registrar");

        Set<String> seenProviders = new HashSet<>();
        ClassLoader contextLoader = Thread.currentThread().getContextClassLoader();
        loadFromLoader(providerType, contextLoader, registrar, seenProviders);

        ClassLoader ownerLoader = ownerType.getClassLoader();
        if (ownerLoader != contextLoader) {
            loadFromLoader(providerType, ownerLoader, registrar, seenProviders);
        }
    }

    private static <P> void loadFromLoader(
            Class<P> providerType,
            ClassLoader loader,
            Consumer<P> registrar,
            Set<String> seenProviders) {
        ServiceLoader<P> providers = loader == null
                ? ServiceLoader.load(providerType)
                : ServiceLoader.load(providerType, loader);
        for (P provider : providers) {
            if (provider == null) {
                continue;
            }
            String className = provider.getClass().getName();
            if (!seenProviders.add(className)) {
                continue;
            }
            registrar.accept(provider);
        }
    }
}
