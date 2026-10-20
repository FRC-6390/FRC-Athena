package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.mechanism.persist.PersistentStore;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeSet;

/**
 * Tuning layers and the tuning files served by the ARCP file service.
 *
 * <p>Layering, highest first: the robot store (values changed on the robot, or by dashboards), the deployed
 * tuning file ({@code src/main/deploy/athena/tuning}, copied to {@code /home/lvuser/deploy/athena/tuning}),
 * and the code default. The deployed file is installed as the store's read-only fallback, so the store
 * always wins and a deployed value is never written into the store.</p>
 *
 * <p>Two generated files are refreshed about once a second when they change:</p>
 * <ul>
 *   <li>{@code tuning/diff.txt}: one row per persisted value with the robot, deployed and code-default values
 *       and where the robot value comes from;</li>
 *   <li>{@code tuning/pull.txt}: the robot's persisted values in the store file format. Save it as
 *       {@code src/main/deploy/athena/tuning} to deploy the current tuning.</li>
 * </ul>
 */
final class ArcpTuning {
    static final String DIFF_NAME = "diff.txt";
    static final String PULL_NAME = "pull.txt";

    private final Map<String, Object> deployed;
    private final Path exportDirectory;
    /** Where generated files are queued; null writes them on the calling thread. */
    private final ExportWriter writer;
    private String lastDiff = "";
    private String lastPull = "";

    /**
     * @param deployed deployed tuning entries, as read at boot
     * @param exportDirectory directory the generated files are written to
     */
    ArcpTuning(Map<String, Object> deployed, Path exportDirectory) {
        this(deployed, exportDirectory, null);
    }

    /**
     * @param deployed deployed tuning entries, as read at boot
     * @param exportDirectory directory the generated files are written to
     * @param writer background writer for the generated files, or null to write them synchronously
     */
    ArcpTuning(Map<String, Object> deployed, Path exportDirectory, ExportWriter writer) {
        this.deployed = Map.copyOf(Objects.requireNonNull(deployed, "deployed"));
        this.exportDirectory = Objects.requireNonNull(exportDirectory, "exportDirectory");
        this.writer = writer;
    }

    /** Reads a deployed tuning file. A missing file is an empty layer. */
    static Map<String, Object> readDeployed(Path file) {
        return file == null ? Map.of() : PersistentStore.readTuning(file);
    }

    /**
     * Regenerates the diff and pull files when their contents change.
     *
     * @param robot current persisted values of the robot
     * @param codeDefaults code defaults of the persisted values
     * @return true when a file was written
     */
    boolean refresh(Map<String, Object> robot, Map<String, Object> codeDefaults) {
        String diff = diff(robot, codeDefaults);
        String pull = PersistentStore.renderTuning(robot);
        boolean changed = false;
        if (!diff.equals(lastDiff)) {
            write(DIFF_NAME, diff);
            lastDiff = diff;
            changed = true;
        }
        if (!pull.equals(lastPull)) {
            write(PULL_NAME, pull);
            lastPull = pull;
            changed = true;
        }
        return changed;
    }

    /** Tab-separated diff: {@code key robot deployed code source}. */
    String diff(Map<String, Object> robot, Map<String, Object> codeDefaults) {
        Set<String> keys = new TreeSet<>();
        keys.addAll(robot.keySet());
        keys.addAll(deployed.keySet());
        keys.addAll(codeDefaults.keySet());
        StringBuilder out = new StringBuilder("# athena tuning diff: key, robot, deployed, code default, source\n");
        for (String key : keys) {
            Object robotValue = robot.get(key);
            Object deployedValue = deployed.get(key);
            Object codeValue = codeDefaults.get(key);
            String source;
            if (robotValue == null) {
                source = "not-on-robot";
            } else if (deployedValue != null && robotValue.equals(deployedValue)) {
                source = "deployed";
            } else if (deployedValue == null && codeValue != null && robotValue.equals(codeValue)) {
                source = "code";
            } else if (deployedValue == null && codeValue == null) {
                source = "robot";
            } else {
                source = "robot-changed";
            }
            out.append(key).append('\t')
                    .append(robotValue == null ? "-" : robotValue).append('\t')
                    .append(deployedValue == null ? "-" : deployedValue).append('\t')
                    .append(codeValue == null ? "-" : codeValue).append('\t')
                    .append(source).append('\n');
        }
        return out.toString();
    }

    /**
     * Writes a file with a temporary file and an atomic rename.
     *
     * @param directory target directory, created when missing
     * @param name file name
     * @param contents UTF-8 contents
     */
    static void writeFile(Path directory, String name, String contents) {
        try {
            Files.createDirectories(directory);
            Path target = directory.resolve(name);
            Path temp = directory.resolve(name + ".tmp");
            Files.write(temp, contents.getBytes(StandardCharsets.UTF_8));
            try {
                Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException unsupported) {
                Files.move(temp, target, StandardCopyOption.REPLACE_EXISTING);
            }
        } catch (IOException exception) {
            throw new UncheckedIOException("Unable to write export " + name, exception);
        }
    }

    private void write(String name, String contents) {
        if (writer != null) {
            writer.write(exportDirectory, name, contents);
        } else {
            writeFile(exportDirectory, name, contents);
        }
    }
}
