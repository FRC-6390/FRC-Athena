package ca.frc6390.athena.core.auto;

import java.util.List;
import java.util.Optional;
import java.util.ServiceLoader;
import java.util.Comparator;

import ca.frc6390.athena.core.RobotAuto;

/**
 * Discovers available {@link AutoBackend} implementations and selects the one that supports a
 * requested {@link RobotAuto.AutoSource}.
 */
public final class AutoBackends {
    private static final List<AutoBackend> BACKENDS =
            ServiceLoader.load(AutoBackend.class).stream().map(ServiceLoader.Provider::get).toList();

    private AutoBackends() {
    }

    public static Optional<AutoBackend> forSource(RobotAuto.AutoSource source) {
        return BACKENDS.stream()
                .filter(b -> b.supports(source))
                .max(Comparator.comparingInt(b -> b.priority(source)));
    }
}
