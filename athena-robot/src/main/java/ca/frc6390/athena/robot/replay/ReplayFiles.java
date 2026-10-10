package ca.frc6390.athena.robot.replay;

import java.io.IOException;
import java.nio.file.FileStore;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * File management for {@code .athrp} replay files: naming, listing, deletion, and retention.
 *
 * <p>File names are {@code <event>-<type><number>-<yyyyMMdd-HHmmss>.athrp} when FMS or driver station match
 * information exists, and {@code practice-<yyyyMMdd-HHmmss>.athrp} otherwise.</p>
 */
public final class ReplayFiles {
    public static final String SUFFIX = ".athrp";
    private static final String PRACTICE = "practice";
    private static final DateTimeFormatter TIMESTAMP = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");
    private static final int TIMESTAMP_LENGTH = 15;

    /**
     * A replay file with the identity parsed from its name.
     *
     * @param name file name
     * @param sizeBytes size in bytes
     * @param modified last modification time
     * @param timestamp timestamp from the name, empty when the name does not match the convention
     * @param event event name, empty for practice files
     * @param matchType type code: p, q, e, or m; empty for practice files
     * @param matchNumber match number, 0 for practice files
     * @param practice true when the file is not a qualification or elimination match
     */
    public record FileInfo(
            String name,
            long sizeBytes,
            Instant modified,
            String timestamp,
            String event,
            String matchType,
            int matchNumber,
            boolean practice) {
    }

    private final StoredFiles files;

    private ReplayFiles(Path directory) {
        this.files = StoredFiles.of(directory, SUFFIX);
    }

    /**
     * Returns the replay file category for a directory.
     *
     * @param directory replay directory
     * @return replay files
     */
    public static ReplayFiles at(Path directory) {
        return new ReplayFiles(directory);
    }

    /** Returns the underlying generic stored-file category. */
    public StoredFiles stored() {
        return files;
    }

    /** Lists replay files sorted by name, with identity parsed from each name. */
    public List<FileInfo> list() {
        List<FileInfo> infos = new ArrayList<>();
        for (StoredFiles.Entry entry : files.list()) {
            infos.add(parse(entry));
        }
        return List.copyOf(infos);
    }

    /**
     * Deletes one replay file.
     *
     * @param name file name from {@link #list()}
     * @return true when deleted
     */
    public boolean delete(String name) {
        return files.delete(name);
    }

    /**
     * Copies one replay file to a target path, for example a USB stick or a laptop. The copy is written to a
     * temporary sibling and moved into place, so a partial copy never has the final name.
     *
     * @param name file name from {@link #list()}
     * @param target destination file, or an existing directory to copy into under the same name
     * @return the path written
     * @throws IOException when the source is missing or the copy fails
     */
    public Path export(String name, Path target) throws IOException {
        Path source = files.resolve(name);
        if (!Files.isRegularFile(source)) {
            throw new java.nio.file.NoSuchFileException(source.toString());
        }
        Path destination = target.toAbsolutePath().normalize();
        if (Files.isDirectory(destination)) {
            destination = destination.resolve(name);
        }
        Path parent = destination.getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }
        Path temporary = destination.resolveSibling(destination.getFileName() + ".tmp");
        Files.copy(source, temporary, StandardCopyOption.REPLACE_EXISTING);
        try {
            Files.move(temporary, destination, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } catch (IOException atomicUnsupported) {
            Files.move(temporary, destination, StandardCopyOption.REPLACE_EXISTING);
        }
        return destination;
    }

    /** Returns the file name for a match identity and wall-clock time. */
    public static String fileName(ReplayMatch match, LocalDateTime time) {
        String stamp = TIMESTAMP.format(time);
        if (!match.identified()) {
            return PRACTICE + "-" + stamp + SUFFIX;
        }
        String event = match.eventName().isEmpty() ? "unknown" : sanitize(match.eventName());
        return event + "-" + match.typeCode() + Math.max(0, match.matchNumber()) + "-" + stamp + SUFFIX;
    }

    static FileInfo parse(StoredFiles.Entry entry) {
        String name = entry.name();
        String stem = name.substring(0, name.length() - SUFFIX.length());
        if (stem.length() <= TIMESTAMP_LENGTH || stem.charAt(stem.length() - TIMESTAMP_LENGTH - 1) != '-') {
            return new FileInfo(name, entry.sizeBytes(), entry.modified(), "", "", "", 0, true);
        }
        String timestamp = stem.substring(stem.length() - TIMESTAMP_LENGTH);
        String prefix = stem.substring(0, stem.length() - TIMESTAMP_LENGTH - 1);
        if (prefix.equals(PRACTICE)) {
            return new FileInfo(name, entry.sizeBytes(), entry.modified(), timestamp, "", "", 0, true);
        }
        int split = prefix.lastIndexOf('-');
        if (split < 0) {
            return new FileInfo(name, entry.sizeBytes(), entry.modified(), timestamp, prefix, "", 0, true);
        }
        String event = prefix.substring(0, split);
        String match = prefix.substring(split + 1);
        int digits = 0;
        while (digits < match.length() && !Character.isDigit(match.charAt(digits))) {
            digits++;
        }
        String type = match.substring(0, digits);
        int number = 0;
        try {
            number = Integer.parseInt(match.substring(digits));
        } catch (NumberFormatException ignored) {
            // Keep 0 for names that do not follow the convention.
        }
        boolean practice = type.equals("p") || type.equals("m");
        return new FileInfo(name, entry.sizeBytes(), entry.modified(), timestamp, event, type, number, practice);
    }

    /**
     * Enforces retention. Deletes files until every limit holds, never deleting {@code protectedName}.
     *
     * @param retention retention rules
     * @param protectedName file just written
     * @return names of deleted files
     */
    List<String> enforce(Retention retention, String protectedName) {
        List<FileInfo> infos = new ArrayList<>(list());
        infos.sort(Comparator.comparing(FileInfo::timestamp).thenComparing(FileInfo::modified)
                .thenComparing(FileInfo::name));
        List<FileInfo> candidates = new ArrayList<>();
        if (retention.keepsMatchesOverPractice()) {
            infos.stream().filter(FileInfo::practice).forEach(candidates::add);
            infos.stream().filter(info -> !info.practice()).forEach(candidates::add);
        } else {
            candidates.addAll(infos);
        }
        long total = 0;
        for (FileInfo info : infos) total += info.sizeBytes();
        int count = infos.size();
        List<String> deleted = new ArrayList<>();
        for (FileInfo candidate : candidates) {
            if (!exceeds(retention, total, count, freeBytes())) break;
            if (candidate.name().equals(protectedName)) continue;
            if (files.delete(candidate.name())) {
                deleted.add(candidate.name());
                total -= candidate.sizeBytes();
                count--;
            }
        }
        return List.copyOf(deleted);
    }

    private long freeBytes() {
        try {
            FileStore store = Files.getFileStore(files.directory());
            return store.getUsableSpace();
        } catch (IOException | RuntimeException exception) {
            return Long.MAX_VALUE;
        }
    }

    private static boolean exceeds(Retention retention, long total, int count, long free) {
        return (retention.maxTotalBytes() > 0 && total > retention.maxTotalBytes())
                || (retention.maxFiles() > 0 && count > retention.maxFiles())
                || (retention.minimumFreeDiskBytes() > 0 && free < retention.minimumFreeDiskBytes());
    }

    private static String sanitize(String value) {
        return value.replaceAll("[^A-Za-z0-9_.]", "_");
    }
}
