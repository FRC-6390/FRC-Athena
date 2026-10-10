package ca.frc6390.athena.robot.replay;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

/**
 * Directory-based listing, deletion, and resolution for one category of stored files (replays, and later
 * A1 values, B1 models, and health). Names are plain file names: separators and {@code ..} are rejected.
 */
public final class StoredFiles {
    /**
     * One stored file.
     *
     * @param name file name
     * @param sizeBytes size in bytes
     * @param modified last modification time
     */
    public record Entry(String name, long sizeBytes, Instant modified) {
    }

    private final Path directory;
    private final String suffix;

    private StoredFiles(Path directory, String suffix) {
        this.directory = directory.toAbsolutePath().normalize();
        this.suffix = suffix;
    }

    /**
     * Returns a category rooted at a directory.
     *
     * @param directory directory; created lazily when files are written
     * @param suffix file suffix including the dot, such as {@code .athrp}; empty for all files
     * @return stored files
     */
    public static StoredFiles of(Path directory, String suffix) {
        return new StoredFiles(directory, suffix == null ? "" : suffix);
    }

    public Path directory() {
        return directory;
    }

    /** Lists files in this category, sorted by name. A missing directory lists as empty. */
    public List<Entry> list() {
        if (!Files.isDirectory(directory)) {
            return List.of();
        }
        List<Entry> entries = new ArrayList<>();
        try (Stream<Path> files = Files.list(directory)) {
            files.filter(Files::isRegularFile)
                    .filter(path -> path.getFileName().toString().endsWith(suffix))
                    .forEach(path -> entries.add(entry(path)));
        } catch (IOException exception) {
            throw new IllegalStateException("Cannot list " + directory + ".", exception);
        }
        entries.sort(Comparator.comparing(Entry::name));
        return List.copyOf(entries);
    }

    /**
     * Resolves a listed name to a path inside the directory.
     *
     * @param name file name
     * @return path
     */
    public Path resolve(String name) {
        if (name == null || name.isBlank() || name.contains("/") || name.contains("\\") || name.equals("..")
                || name.equals(".") || !name.endsWith(suffix)) {
            throw new IllegalArgumentException("Not a stored file name: " + name);
        }
        return directory.resolve(name);
    }

    /**
     * Deletes one stored file.
     *
     * @param name file name
     * @return true when a file was deleted
     */
    public boolean delete(String name) {
        Path path = resolve(name);
        try {
            return Files.deleteIfExists(path);
        } catch (IOException exception) {
            throw new IllegalStateException("Cannot delete " + path + ".", exception);
        }
    }

    Entry entry(Path path) {
        try {
            return new Entry(
                    path.getFileName().toString(),
                    Files.size(path),
                    Files.getLastModifiedTime(path).toInstant());
        } catch (IOException exception) {
            throw new IllegalStateException("Cannot stat " + path + ".", exception);
        }
    }
}
