package ca.frc6390.athena.wpilib.arcp;

import java.io.IOException;
import java.nio.file.FileStore;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.atomic.AtomicBoolean;

/** Coalesces potentially blocking mount/free-space queries and exposes only a cached result to the robot. */
final class DiskSpaceMonitor implements AutoCloseable {
    @FunctionalInterface interface Probe { long usableBytes() throws IOException; }

    private final Probe probe;
    private final AtomicBoolean scheduled = new AtomicBoolean();
    private final ExecutorService worker = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "Athena-ArcpDiskSpace");
        thread.setDaemon(true);
        thread.setPriority(Thread.MIN_PRIORITY);
        return thread;
    });
    private volatile long usableBytes = -1L;
    private volatile boolean closed;

    DiskSpaceMonitor(Path directory) {
        this(new Probe() {
            private FileStore store;
            @Override public long usableBytes() throws IOException {
                try {
                    if (store == null) store = Files.getFileStore(directory);
                    return store.getUsableSpace();
                } catch (IOException | RuntimeException exception) {
                    store = null;
                    throw exception;
                }
            }
        });
    }

    DiskSpaceMonitor(Probe probe) { this.probe = probe; }

    long usableBytes() { return usableBytes; }

    void request() {
        if (closed || !scheduled.compareAndSet(false, true)) return;
        try {
            worker.execute(() -> {
                try {
                    long measured = probe.usableBytes();
                    if (!closed) usableBytes = measured;
                } catch (IOException | RuntimeException unavailable) {
                    if (!closed) usableBytes = -1L;
                } finally {
                    scheduled.set(false);
                }
            });
        } catch (RejectedExecutionException shutdown) {
            scheduled.set(false);
        }
    }

    @Override public void close() {
        closed = true;
        worker.shutdownNow();
    }
}
