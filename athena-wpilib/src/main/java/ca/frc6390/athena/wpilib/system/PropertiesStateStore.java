package ca.frc6390.athena.wpilib.system;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Optional;
import java.util.Properties;

final class PropertiesStateStore implements SystemTuningStateStore {
    private final Path path;

    PropertiesStateStore(Path path) {
        this.path = path;
    }

    @Override
    public Optional<SystemTuningState> load() {
        if (!Files.isReadable(path)) return Optional.empty();
        Properties properties = new Properties();
        try (InputStream input = Files.newInputStream(path)) {
            properties.load(input);
            return Optional.of(new SystemTuningState(
                    properties.getProperty("vm.overcommit_memory", ""),
                    properties.getProperty("vm.swappiness", ""),
                    Boolean.parseBoolean(properties.getProperty("webServerRunning", "false")),
                    Boolean.parseBoolean(properties.getProperty("athenaZram", "false")),
                    Boolean.parseBoolean(properties.getProperty("athenaSwapFile", "false"))));
        } catch (IOException | IllegalArgumentException ignored) {
            return Optional.empty();
        }
    }

    @Override
    public SystemAccess.Result save(SystemTuningState state) {
        Properties properties = new Properties();
        properties.setProperty("version", "1");
        properties.setProperty("vm.overcommit_memory", state.overcommitMemory());
        properties.setProperty("vm.swappiness", state.swappiness());
        properties.setProperty("webServerRunning", Boolean.toString(state.webServerRunning()));
        properties.setProperty("athenaZram", Boolean.toString(state.athenaZram()));
        properties.setProperty("athenaSwapFile", Boolean.toString(state.athenaSwapFile()));
        Path temporary = path.resolveSibling(path.getFileName() + ".tmp");
        try {
            Files.createDirectories(path.getParent());
            try (OutputStream output = Files.newOutputStream(temporary)) {
                properties.store(output, "Athena system tuning state");
            }
            try {
                Files.move(temporary, path, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } catch (java.nio.file.AtomicMoveNotSupportedException ignored) {
                Files.move(temporary, path, StandardCopyOption.REPLACE_EXISTING);
            }
            return SystemAccess.Result.success("Saved " + path);
        } catch (IOException exception) {
            return SystemAccess.Result.failure("Cannot save " + path + ": " + exception.getMessage());
        }
    }

    @Override
    public SystemAccess.Result delete() {
        try {
            Files.deleteIfExists(path);
            return SystemAccess.Result.success("Removed " + path);
        } catch (IOException exception) {
            return SystemAccess.Result.failure("Cannot remove " + path + ": " + exception.getMessage());
        }
    }
}
