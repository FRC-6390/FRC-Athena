package ca.frc6390.athena.core.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import ca.frc6390.athena.core.examples.ServiceLoaderRegistryExamples;
import ca.frc6390.athena.core.registry.fixtures.AlphaProvider;
import ca.frc6390.athena.core.registry.fixtures.BetaProvider;
import ca.frc6390.athena.core.registry.fixtures.DemoProvider;
import org.junit.jupiter.api.Test;

final class ServiceLoaderProviderSemanticsTest {

    @Test
    void providerDiscoveryDeduplicatesClassNamesAcrossContextAndOwnerLoaders() throws Exception {
        ClassLoader originalContext = Thread.currentThread().getContextClassLoader();
        URL classesUrl = AlphaProvider.class.getProtectionDomain().getCodeSource().getLocation();

        try (ChildFirstProviderLoader loader = new ChildFirstProviderLoader(
                new URL[] {classesUrl},
                originalContext,
                Set.of(AlphaProvider.class.getName(), BetaProvider.class.getName()))) {
            Thread.currentThread().setContextClassLoader(loader);

            List<DemoProvider> providers = ProviderProbe.discover(DemoProvider.class, ServiceLoaderProviderSemanticsTest.class);
            List<String> providerNames = providers.stream().map(p -> p.getClass().getName()).toList();

            assertEquals(2, providerNames.size());
            assertEquals(Set.of(AlphaProvider.class.getName(), BetaProvider.class.getName()), Set.copyOf(providerNames));

            Map<String, Long> byClass = providerNames.stream()
                    .collect(Collectors.groupingBy(name -> name, Collectors.counting()));
            assertTrue(byClass.values().stream().allMatch(count -> count == 1L));
            assertTrue(providers.stream().anyMatch(p -> p.getClass().getClassLoader() == loader));
        } finally {
            Thread.currentThread().setContextClassLoader(originalContext);
        }
    }

    @Test
    void exampleHelperSurfacesDiscoveredProviderNames() {
        List<String> names = ServiceLoaderRegistryExamples.discoverProviderClassNames(
                DemoProvider.class,
                ServiceLoaderProviderSemanticsTest.class);

        assertEquals(Set.of(AlphaProvider.class.getName(), BetaProvider.class.getName()), Set.copyOf(names));
    }

    private static final class ProviderProbe extends PluginRegistryBase<Object> {
        private ProviderProbe() {}

        static <P> List<P> discover(Class<P> providerType, Class<?> ownerType) {
            List<P> providers = new ArrayList<>();
            load(providerType, ownerType, providers::add);
            return providers;
        }

        static <P> void load(Class<P> providerType, Class<?> ownerType, Consumer<P> registrar) {
            loadProviders(providerType, ownerType, registrar);
        }

        @Override
        protected String missingMessage(String key) {
            return key;
        }
    }

    private static final class ChildFirstProviderLoader extends URLClassLoader {
        private final Set<String> childFirstClasses;

        private ChildFirstProviderLoader(URL[] urls, ClassLoader parent, Set<String> childFirstClasses) {
            super(urls, parent);
            this.childFirstClasses = childFirstClasses;
        }

        @Override
        protected synchronized Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
            if (childFirstClasses.contains(name)) {
                Class<?> loaded = findLoadedClass(name);
                if (loaded == null) {
                    try {
                        loaded = findClass(name);
                    } catch (ClassNotFoundException ignored) {
                        loaded = super.loadClass(name, false);
                    }
                }
                if (resolve) {
                    resolveClass(loaded);
                }
                return loaded;
            }
            return super.loadClass(name, resolve);
        }
    }
}
