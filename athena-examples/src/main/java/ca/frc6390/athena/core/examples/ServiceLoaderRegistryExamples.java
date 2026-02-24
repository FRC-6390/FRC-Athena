package ca.frc6390.athena.core.examples;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import ca.frc6390.athena.core.registry.PluginRegistryBase;

/**
 * Example helpers describing ServiceLoader-backed provider discovery behavior.
 */
public final class ServiceLoaderRegistryExamples {
    private ServiceLoaderRegistryExamples() {}

    public static <P> List<String> discoverProviderClassNames(Class<P> providerType, Class<?> ownerType) {
        List<String> names = new ArrayList<>();
        ProviderProbe.load(providerType, ownerType, provider -> names.add(provider.getClass().getName()));
        return names;
    }

    public static <P> int countProviders(Class<P> providerType, Class<?> ownerType) {
        return discoverProviderClassNames(providerType, ownerType).size();
    }

    private static final class ProviderProbe extends PluginRegistryBase<Object> {
        static <P> void load(Class<P> providerType, Class<?> ownerType, Consumer<P> registrar) {
            loadProviders(providerType, ownerType, registrar);
        }

        @Override
        protected String missingMessage(String key) {
            return key;
        }
    }
}
