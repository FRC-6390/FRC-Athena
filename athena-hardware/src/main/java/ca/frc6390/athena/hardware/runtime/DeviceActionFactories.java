package ca.frc6390.athena.hardware.runtime;

import java.util.ServiceLoader;

/**
 * Process-wide access to the {@link DeviceActionFactory} supplied by athena-mechanisms. The implementation
 * is discovered once through {@link ServiceLoader}; without athena-mechanisms on the classpath the lookup
 * fails with a clear message.
 */
public final class DeviceActionFactories {
    private static volatile DeviceActionFactory factory;

    private DeviceActionFactories() {
    }

    /**
     * Returns the installed device action factory.
     *
     * @return factory
     * @throws IllegalStateException when no implementation is on the classpath
     */
    public static DeviceActionFactory get() {
        DeviceActionFactory found = factory;
        if (found == null) {
            found = ServiceLoader.load(DeviceActionFactory.class)
                    .findFirst()
                    .orElseThrow(() -> new IllegalStateException(
                            "Device actions require athena-mechanisms on the classpath."));
            factory = found;
        }
        return found;
    }
}
