package ca.frc6390.athena.mechanism.persist;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.TreeMap;

/**
 * A small robot-side key/value store for persisted tuning and telemetry values.
 *
 * <p>Keys are telemetry paths (mechanism tree path plus field path). Values are typed: {@code double},
 * {@code long}, {@code boolean}, {@code String}, and {@code double[]}. The store lives in memory and is
 * written to one text file in its directory. Writes are batched: {@link #flushIfDue(double)} writes at
 * most once per second, and {@link #flush()} writes immediately. A flush writes a temporary file and
 * atomically renames it over the store file, so a power loss leaves either the old or the new contents.</p>
 *
 * <p>The store has no dependency on WPILib or the hardware HAL, so it can be used in tests with a
 * temporary directory.</p>
 */
public final class PersistentStore implements AutoCloseable {
    /** Name of the store file inside the store directory. */
    public static final String FILE_NAME = "athena-store.txt";
    /** Minimum interval between batched flushes, in seconds. */
    public static final double FLUSH_PERIOD_SECONDS = 1.0;

    private static final String HEADER = "# athena persistent store v1";
    private static final String TEMP_NAME = FILE_NAME + ".tmp";

    private final Path directory;
    private final Path file;
    private final TreeMap<String, Object> entries = new TreeMap<>();
    /** Read-only lower layer (the deployed tuning file). Entries here are never written to the store file. */
    private Map<String, Object> fallback = Map.of();
    private boolean dirty;
    private double lastFlushSeconds = Double.NaN;
    private String lastError = "";
    private int skippedLines;
    private long generation;
    private final Object writerLock = new Object();
    private final SnapshotWriter writer;
    private final java.util.function.LongSupplier nanoTime;
    private java.util.concurrent.ExecutorService executor;
    private boolean scheduled;
    private boolean closed;
    private long retryAfterNanos;
    private boolean retryPending;

    @FunctionalInterface
    interface SnapshotWriter { void write(Path target, byte[] bytes) throws IOException; }

    private PersistentStore(Path directory, SnapshotWriter writer, java.util.function.LongSupplier nanoTime) {
        this.directory = directory;
        this.file = directory.resolve(FILE_NAME);
        this.writer = writer;
        this.nanoTime = nanoTime;
    }

    /**
     * Opens the store in a directory, creating the directory when needed and loading existing entries.
     * Malformed lines are skipped and counted in {@link #skippedLines()}.
     *
     * @param directory store directory
     * @return opened store
     * @throws UncheckedIOException when the directory cannot be created or the file cannot be read
     */
    public static PersistentStore open(Path directory) {
        return open(directory, PersistentStore::writeAtomically);
    }

    static PersistentStore open(Path directory, SnapshotWriter writer) {
        return open(directory, writer, System::nanoTime);
    }

    static PersistentStore open(Path directory, SnapshotWriter writer, java.util.function.LongSupplier nanoTime) {
        Path safeDirectory = Objects.requireNonNull(directory, "directory").toAbsolutePath().normalize();
        try {
            Files.createDirectories(safeDirectory);
        } catch (IOException exception) {
            throw new UncheckedIOException("Unable to create persistent store directory " + safeDirectory, exception);
        }
        PersistentStore store = new PersistentStore(safeDirectory, Objects.requireNonNull(writer, "writer"),
                Objects.requireNonNull(nanoTime, "nanoTime"));
        store.load();
        return store;
    }

    /** Returns the store directory. */
    public Path directory() {
        return directory;
    }

    /** Returns the store file path. */
    public Path file() {
        return file;
    }

    /** Returns the number of lines ignored while loading because they were malformed. */
    public synchronized int skippedLines() {
        return skippedLines;
    }

    /** Returns the message of the last failed flush, or an empty string when the last flush succeeded. */
    public synchronized String lastError() {
        return lastError;
    }

    /** Returns true when in-memory entries differ from the file. */
    public synchronized boolean dirty() {
        return dirty;
    }

    /**
     * Installs a read-only lower layer. Keys found here are returned by {@link #get} when the store has no
     * entry of its own, so the robot store always wins over the layer. Layer entries are not written to the file.
     *
     * @param layer entries, for example read with {@link #readTuning}; copied
     */
    public synchronized void fallback(Map<String, Object> layer) {
        fallback = Map.copyOf(Objects.requireNonNull(layer, "layer"));
    }

    /** Returns the read-only lower layer as installed by {@link #fallback}. */
    public synchronized Map<String, Object> fallback() {
        return fallback;
    }

    /** Returns the number of stored entries. */
    public synchronized int size() {
        return entries.size();
    }

    /** Returns the stored keys in sorted order. */
    public synchronized Set<String> keys() {
        return Collections.unmodifiableSet(new LinkedHashSet<>(entries.keySet()));
    }

    /**
     * Returns the stored value. Values are {@link Double}, {@link Long}, {@link Boolean}, {@link String},
     * or a copy of a {@code double[]}.
     *
     * @param key entry key
     * @return stored value, or empty when absent
     */
    public synchronized Optional<Object> get(String key) {
        Object value = lookup(requireKey(key));
        if (value instanceof double[] array) {
            return Optional.of(array.clone());
        }
        return Optional.ofNullable(value);
    }

    /** Returns the stored {@code double} value, or empty when absent or of another type. */
    public synchronized OptionalDouble getDouble(String key) {
        Object value = lookup(requireKey(key));
        return value instanceof Double number ? OptionalDouble.of(number) : OptionalDouble.empty();
    }

    /** Stores a finite {@code double}. Returns true when the stored value changed. */
    public synchronized boolean put(String key, double value) {
        requireFinite(value);
        return set(requireKey(key), value);
    }

    /** Stores a {@code long}. Returns true when the stored value changed. */
    public synchronized boolean put(String key, long value) {
        return set(requireKey(key), value);
    }

    /** Stores a {@code boolean}. Returns true when the stored value changed. */
    public synchronized boolean put(String key, boolean value) {
        return set(requireKey(key), value);
    }

    /** Stores a {@code String}. Returns true when the stored value changed. */
    public synchronized boolean put(String key, String value) {
        return set(requireKey(key), Objects.requireNonNull(value, "value"));
    }

    /** Stores a copy of a finite {@code double} array. Returns true when the stored value changed. */
    public synchronized boolean put(String key, double[] values) {
        Objects.requireNonNull(values, "values");
        for (double value : values) {
            requireFinite(value);
        }
        return set(requireKey(key), values.clone());
    }

    /** Removes an entry. Returns true when it existed. */
    public synchronized boolean remove(String key) {
        requireOpen();
        if (entries.remove(requireKey(key)) == null) {
            return false;
        }
        dirty = true;
        generation++;
        return true;
    }

    /**
     * Removes every entry whose key is not in {@code keep}. Used at boot so values for fields that were
     * renamed or removed do not linger.
     *
     * @param keep keys to keep
     * @return number of removed entries
     */
    public synchronized int retainOnly(Set<String> keep) {
        requireOpen();
        Objects.requireNonNull(keep, "keep");
        int removed = 0;
        for (String key : List.copyOf(entries.keySet())) {
            if (!keep.contains(key)) {
                entries.remove(key);
                removed++;
            }
        }
        if (removed > 0) {
            dirty = true;
            generation++;
        }
        return removed;
    }

    /**
     * Writes pending changes when at least {@link #FLUSH_PERIOD_SECONDS} passed since the last batched
     * flush. The first pending change is written immediately.
     *
     * @param nowSeconds current robot timestamp
     * @return true when a flush was written
     */
    public boolean flushIfDue(double nowSeconds) {
        synchronized (this) {
            if (!due(nowSeconds)) return false;
            lastFlushSeconds = nowSeconds;
        }
        return flush();
    }

    /** Queues one coalesced background write when the batching interval permits; never performs disk I/O. */
    public synchronized boolean requestFlushIfDue(double nowSeconds) {
        if (!due(nowSeconds)) return false;
        boolean accepted = requestFlush();
        if (accepted) lastFlushSeconds = nowSeconds;
        return accepted;
    }

    private boolean due(double nowSeconds) {
        return dirty && (Double.isNaN(lastFlushSeconds) || !Double.isFinite(nowSeconds)
                || nowSeconds < lastFlushSeconds || nowSeconds - lastFlushSeconds >= FLUSH_PERIOD_SECONDS);
    }

    /**
     * Queues the latest state for background persistence. Updates made during a write coalesce into its next
     * snapshot. Failures retain dirty state and are retried by later requests, with a one-second retry backoff.
     *
     * @return true when a write is queued or already in progress
     */
    public synchronized boolean requestFlush() {
        if (closed || !dirty || retryPending && nanoTime.getAsLong() - retryAfterNanos < 0L) return false;
        if (scheduled) return true;
        if (executor == null) executor = java.util.concurrent.Executors.newSingleThreadExecutor(runnable -> {
            Thread worker = new Thread(runnable, "Athena-PersistentStore");
            worker.setDaemon(true);
            return worker;
        });
        scheduled = true;
        executor.execute(this::drain);
        return true;
    }

    private void drain() {
        while (true) {
            boolean success = flush();
            synchronized (this) {
                if (!success && !lastError.isEmpty()) {
                    retryAfterNanos = nanoTime.getAsLong() + 1_000_000_000L;
                    retryPending = true;
                }
                if (!success || !dirty || closed) {
                    scheduled = false;
                    notifyAll();
                    return;
                }
            }
        }
    }

    /**
     * Writes a versioned snapshot durably now. Intended for explicit off-loop callers and shutdown; use
     * {@link #requestFlush()} from periodic code. Rendering, file writes, force and rename never hold the store
     * monitor, so concurrent reads and mutations remain available even when storage is blocked.
     *
     * @return true when nothing is pending or this snapshot was successfully written
     */
    public boolean flush() {
        synchronized (writerLock) {
            final Map<String, Object> snapshot;
            final long version;
            synchronized (this) {
                if (!dirty) return true;
                snapshot = new TreeMap<>(entries);
                version = generation;
            }
            try {
                writer.write(file, renderEntries(snapshot).getBytes(StandardCharsets.UTF_8));
                synchronized (this) {
                    if (generation == version) dirty = false;
                    lastError = "";
                    retryPending = false;
                }
                return true;
            } catch (IOException | RuntimeException exception) {
                synchronized (this) {
                    lastError = exception.getClass().getSimpleName() + ": " + exception.getMessage();
                }
                return false;
            }
        }
    }

    static void writeAtomically(Path file, byte[] bytes) throws IOException {
        Path temp = file.resolveSibling(TEMP_NAME);
        try (FileChannel channel = FileChannel.open(temp,
                StandardOpenOption.CREATE, StandardOpenOption.WRITE, StandardOpenOption.TRUNCATE_EXISTING)) {
            writeFully(channel, ByteBuffer.wrap(bytes));
            channel.force(true);
        }
        try {
            Files.move(temp, file, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
        } catch (AtomicMoveNotSupportedException unsupported) {
            Files.move(temp, file, StandardCopyOption.REPLACE_EXISTING);
        }
    }

    static void writeFully(java.nio.channels.WritableByteChannel channel, ByteBuffer bytes) throws IOException {
        while (bytes.hasRemaining()) channel.write(bytes);
    }

    /** Stops accepting background work and durably writes the final state. Call off the robot thread. */
    @Override
    public void close() {
        java.util.concurrent.ExecutorService active;
        synchronized (this) {
            if (closed) return;
            closed = true;
            active = executor;
        }
        try {
            if (!flush()) throw new IllegalStateException("Unable to flush persistent store on close: " + lastError());
        } finally {
            if (active != null) active.shutdown();
        }
    }

    private void requireOpen() {
        if (closed) throw new IllegalStateException("Persistent store is closed.");
    }

    private boolean set(String key, Object value) {
        requireOpen();
        Object previous = entries.get(key);
        if (previous != null && sameValue(previous, value)) {
            return false;
        }
        entries.put(key, value);
        dirty = true;
        generation++;
        return true;
    }

    private void load() {
        if (!Files.isRegularFile(file)) {
            return;
        }
        List<String> lines;
        try {
            lines = Files.readAllLines(file, StandardCharsets.UTF_8);
        } catch (IOException exception) {
            throw new UncheckedIOException("Unable to read persistent store " + file, exception);
        }
        for (String line : lines) {
            if (line.isBlank() || line.startsWith("#")) {
                continue;
            }
            try {
                parseLine(line);
            } catch (RuntimeException exception) {
                skippedLines++;
            }
        }
    }

    private void parseLine(String line) {
        var parsed = parseEntry(line);
        entries.put(parsed.getKey(), parsed.getValue());
    }

    private static Map.Entry<String, Object> parseEntry(String line) {
        String[] parts = line.split("\t", -1);
        if (parts.length != 3) {
            throw new IllegalArgumentException("expected three fields");
        }
        String key = unescape(parts[0]);
        Object value = switch (parts[1]) {
            case "D" -> finite(Double.parseDouble(parts[2]));
            case "L" -> Long.parseLong(parts[2]);
            case "B" -> switch (parts[2]) {
                case "true" -> Boolean.TRUE;
                case "false" -> Boolean.FALSE;
                default -> throw new IllegalArgumentException("invalid boolean");
            };
            case "S" -> unescape(parts[2]);
            case "A" -> parseArray(parts[2]);
            default -> throw new IllegalArgumentException("unknown type tag");
        };
        requireKey(key);
        return Map.entry(key, value);
    }

    private static double[] parseArray(String text) {
        if (text.isEmpty()) {
            return new double[0];
        }
        String[] tokens = text.split(",", -1);
        double[] values = new double[tokens.length];
        for (int index = 0; index < tokens.length; index++) {
            values[index] = finite(Double.parseDouble(tokens[index]));
        }
        return values;
    }

    private Object lookup(String key) {
        Object value = entries.get(key);
        return value != null ? value : fallback.get(key);
    }

    /**
     * Reads a file written in the store format (for example a deployed tuning file). Missing files give an empty
     * map; malformed lines are skipped.
     *
     * @param file file to read
     * @return entries in key order
     */
    public static Map<String, Object> readTuning(Path file) {
        TreeMap<String, Object> out = new TreeMap<>();
        if (file == null || !Files.isRegularFile(file)) {
            return out;
        }
        try {
            for (String line : Files.readAllLines(file, StandardCharsets.UTF_8)) {
                if (line.isBlank() || line.startsWith("#")) {
                    continue;
                }
                try {
                    var entry = parseEntry(line);
                    out.put(entry.getKey(), entry.getValue());
                } catch (RuntimeException malformed) {
                    // Skipped, like the store loader does.
                }
            }
        } catch (IOException exception) {
            throw new UncheckedIOException("Unable to read tuning file " + file, exception);
        }
        return out;
    }

    /**
     * Renders entries in the store file format, with the store header. Used for tuning exports.
     *
     * @param entries entries to render, with values of the types the store supports
     * @return file contents
     */
    public static String renderTuning(Map<String, Object> entries) {
        TreeMap<String, Object> sorted = new TreeMap<>(entries);
        return renderEntries(sorted);
    }

    private static String renderEntries(Map<String, Object> entries) {
        StringBuilder builder = new StringBuilder(HEADER).append('\n');
        for (var entry : entries.entrySet()) {
            builder.append(escape(entry.getKey())).append('\t');
            Object value = entry.getValue();
            if (value instanceof Double number) {
                builder.append("D\t").append(Double.toString(number));
            } else if (value instanceof Long number) {
                builder.append("L\t").append(number);
            } else if (value instanceof Boolean flag) {
                builder.append("B\t").append(flag);
            } else if (value instanceof String text) {
                builder.append("S\t").append(escape(text));
            } else if (value instanceof double[] array) {
                builder.append("A\t");
                for (int index = 0; index < array.length; index++) {
                    if (index > 0) {
                        builder.append(',');
                    }
                    builder.append(Double.toString(array[index]));
                }
            }
            builder.append('\n');
        }
        return builder.toString();
    }

    private static boolean sameValue(Object left, Object right) {
        if (left instanceof double[] leftArray && right instanceof double[] rightArray) {
            return Arrays.equals(leftArray, rightArray);
        }
        return left.equals(right);
    }

    private static String requireKey(String key) {
        Objects.requireNonNull(key, "key");
        if (key.isBlank()) {
            throw new IllegalArgumentException("Persistent store key must not be blank.");
        }
        return key;
    }

    private static void requireFinite(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Persistent store numbers must be finite.");
        }
    }

    private static double finite(double value) {
        requireFinite(value);
        return value;
    }

    static String escape(String text) {
        StringBuilder builder = new StringBuilder(text.length());
        for (int index = 0; index < text.length(); index++) {
            char character = text.charAt(index);
            switch (character) {
                case '\\' -> builder.append("\\\\");
                case '\t' -> builder.append("\\t");
                case '\n' -> builder.append("\\n");
                case '\r' -> builder.append("\\r");
                default -> builder.append(character);
            }
        }
        return builder.toString();
    }

    static String unescape(String text) {
        StringBuilder builder = new StringBuilder(text.length());
        for (int index = 0; index < text.length(); index++) {
            char character = text.charAt(index);
            if (character != '\\' || index + 1 >= text.length()) {
                builder.append(character);
                continue;
            }
            char next = text.charAt(++index);
            builder.append(switch (next) {
                case 't' -> '\t';
                case 'n' -> '\n';
                case 'r' -> '\r';
                case '\\' -> '\\';
                default -> throw new IllegalArgumentException("invalid escape");
            });
        }
        return builder.toString();
    }
}
