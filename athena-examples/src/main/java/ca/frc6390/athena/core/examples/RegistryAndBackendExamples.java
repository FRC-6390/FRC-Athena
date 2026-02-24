package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.auto.AutoBackend;
import ca.frc6390.athena.core.registry.PluginRegistryBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Example patterns for registry and backend selection behavior in core runtime wiring.
 */
public final class RegistryAndBackendExamples {
    private RegistryAndBackendExamples() {}

    public static final class ExampleRegistry extends PluginRegistryBase<String> {
        @Override
        protected String missingMessage(String key) {
            return "Missing key '" + key + "'";
        }

        public ExampleRegistry add(String key, String value) {
            put(key, value);
            return this;
        }

        public String requireValue(String key) {
            return require(key);
        }
    }

    public static Optional<AutoBackend> selectPreferredBackend(
            List<AutoBackend> backends,
            RobotAuto.AutoSource source) {
        if (backends == null || backends.isEmpty() || source == null) {
            return Optional.empty();
        }
        return backends.stream()
                .filter(backend -> backend != null && backend.supports(source))
                .max(Comparator.comparingInt(backend -> backend.priority(source)));
    }

    public static final class ExampleBackend implements AutoBackend {
        private final RobotAuto.AutoSource source;
        private final int priority;

        public ExampleBackend(RobotAuto.AutoSource source, int priority) {
            this.source = source;
            this.priority = priority;
        }

        @Override
        public boolean supports(RobotAuto.AutoSource source) {
            return this.source == source;
        }

        @Override
        public int priority(RobotAuto.AutoSource source) {
            return supports(source) ? priority : Integer.MIN_VALUE;
        }

        @Override
        public Optional<Command> warmupCommand(RobotAuto.AutoSource source) {
            if (!supports(source)) {
                return Optional.empty();
            }
            return Optional.of(Commands.none());
        }
    }
}
