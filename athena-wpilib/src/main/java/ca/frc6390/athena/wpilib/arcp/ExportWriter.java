package ca.frc6390.athena.wpilib.arcp;

import edu.wpi.first.wpilibj.DriverStation;
import java.nio.file.Path;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * Writes generated ARCP export files (tuning diff and pull, status) on one background thread, so the robot thread never
 * waits on file I/O. Writes to the same file coalesce: only the latest contents queued for a file are written, and
 * writes run in the order their files were first queued.
 *
 * <p>Thread-safe. The robot thread only builds the contents and calls {@link #write}; the writer thread touches the
 * export directories and reports failures through {@link DriverStation}, which is safe from any thread.</p>
 */
final class ExportWriter implements AutoCloseable {
    private static final long CLOSE_TIMEOUT_MILLIS = 2_000L;

    /** Latest contents queued for one target file. */
    private record Pending(Path directory, String name, String contents) {
    }

    private final Object lock = new Object();
    /** Keyed by target file, so a second write to a queued file replaces its contents in place. */
    private final Map<Path, Pending> pending = new LinkedHashMap<>();
    private final ExecutorService thread;
    /** True while a drain task is queued or running. Guarded by {@link #lock}. */
    private boolean scheduled;
    private boolean closed;
    /** Last failure message reported, so a repeating failure is reported once. Guarded by {@link #lock}. */
    private String lastFailure = "";

    ExportWriter() {
        thread = Executors.newSingleThreadExecutor(runnable -> {
            Thread worker = new Thread(runnable, "athena-arcp-export");
            worker.setDaemon(true);
            return worker;
        });
    }

    /**
     * Queues a file for writing. Returns immediately. After {@link #close} the file is written on the calling thread.
     *
     * @param directory target directory
     * @param name file name
     * @param contents UTF-8 contents
     */
    void write(Path directory, String name, String contents) {
        Objects.requireNonNull(directory, "directory");
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(contents, "contents");
        synchronized (lock) {
            if (closed) {
                writeNow(directory, name, contents);
                return;
            }
            pending.put(directory.resolve(name), new Pending(directory, name, contents));
            if (!scheduled) {
                scheduled = true;
                thread.execute(this::drain);
            }
        }
    }

    /**
     * Waits until nothing is queued or being written. Tests call this before reading the files; the robot never does.
     *
     * @param timeoutMillis longest wait
     * @return true when the writer went idle within the timeout
     * @throws InterruptedException when interrupted while waiting
     */
    boolean awaitIdle(long timeoutMillis) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMillis);
        synchronized (lock) {
            while (scheduled || !pending.isEmpty()) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) return false;
                TimeUnit.NANOSECONDS.timedWait(lock, remaining);
            }
            return true;
        }
    }

    /** Flushes what is queued (bounded wait), then stops the writer. Later writes go straight to disk. */
    @Override
    public void close() {
        synchronized (lock) {
            if (closed) return;
            closed = true;
        }
        try {
            awaitIdle(CLOSE_TIMEOUT_MILLIS);
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
        }
        thread.shutdown();
    }

    private void drain() {
        while (true) {
            Pending next;
            synchronized (lock) {
                Iterator<Pending> iterator = pending.values().iterator();
                if (!iterator.hasNext()) {
                    scheduled = false;
                    lock.notifyAll();
                    return;
                }
                next = iterator.next();
                iterator.remove();
            }
            writeNow(next.directory(), next.name(), next.contents());
        }
    }

    private void writeNow(Path directory, String name, String contents) {
        try {
            ArcpTuning.writeFile(directory, name, contents);
        } catch (RuntimeException failure) {
            report(failure);
        }
    }

    private void report(RuntimeException failure) {
        String message = String.valueOf(failure.getMessage());
        synchronized (lock) {
            if (message.equals(lastFailure)) return;
            lastFailure = message;
        }
        DriverStation.reportWarning("Athena ARCP: " + message, false);
    }
}
