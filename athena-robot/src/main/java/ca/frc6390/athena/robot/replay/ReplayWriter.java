package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.robot.RuntimeFailureReporter;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Writes flushed replay data on one low-priority background thread, so the robot thread never compresses or
 * touches the disk.
 *
 * <p>The robot thread hands over a {@link Job} with {@link #submit}: a bounded queue offer. When the queue is
 * full the oldest pending job (never the one being written) is dropped and counted. Drops are reported from the
 * writer thread, not the robot thread. At most {@link #QUEUE_CAPACITY} jobs wait plus one being written are held
 * in RAM. A separate byte budget covers both queued and currently writing jobs; the recorder subtracts
 * these retained bytes from its ring capacity instead of allowing several full histories to accumulate.</p>
 *
 * <p>{@link #awaitIdle(Duration)} blocks until every accepted job has finished, for tests and orderly shutdown.</p>
 */
final class ReplayWriter {
    /** Jobs that may wait for the writer thread. */
    static final int QUEUE_CAPACITY = 2;

    /**
     * One flush, ready to write. Segments and dictionary are immutable byte arrays owned by the job.
     *
     * @param target destination file (name chosen by the recorder)
     * @param metadata METADATA pairs
     * @param dictionary DICTIONARY bytes for the file header
     * @param segments self-contained segments in order
     * @param reason flush reason, for diagnostics
     */
    record Job(Path target, Map<String, String> metadata, byte[] dictionary, List<byte[]> segments, String reason,
               long retainedBytes) {
        Job(Path target, Map<String, String> metadata, byte[] dictionary, List<byte[]> segments, String reason) {
            this(Objects.requireNonNull(target, "target"), metadata, dictionary, List.copyOf(segments), reason,
                    dictionary.length + segments.stream().mapToLong(bytes -> bytes.length).sum());
        }
    }

    private final ArrayBlockingQueue<Job> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    private final RuntimeFailureReporter reporter;
    private final Thread thread;
    private final Object lock = new Object();
    private final Enforcer enforcer;
    private final Consumer<Job> beforeWrite;
    private final long maxRetainedBytes;
    private int pending;
    private long retainedBytes;
    private long recordingEvictions;
    private long reportedRecordingEvictions;
    private long dropped;
    private long reportedDropped;
    private long written;
    private boolean closed;
    private volatile Path lastWritten;

    /** Applies retention after a write; the argument is the file just written, which must be kept. */
    @FunctionalInterface
    interface Enforcer {
        void enforce(String protectedName);
    }

    /**
     * Creates the writer and starts its thread.
     *
     * @param reporter receives write failures and drop reports
     * @param enforcer applies retention after each write
     */
    ReplayWriter(RuntimeFailureReporter reporter, Enforcer enforcer) {
        this(reporter, enforcer, job -> { });
    }

    /** Creates a writer whose hook runs on the writer thread before each job; tests use it to hold the writer. */
    ReplayWriter(RuntimeFailureReporter reporter, Enforcer enforcer, Consumer<Job> beforeWrite) {
        this(reporter, enforcer, beforeWrite, ReplayPolicy.DEFAULT_BUFFER_BYTES + 1024 * 1024);
    }

    ReplayWriter(RuntimeFailureReporter reporter, Enforcer enforcer, Consumer<Job> beforeWrite,
                 long maxRetainedBytes) {
        this.maxRetainedBytes = Math.max(0, maxRetainedBytes);
        this.reporter = Objects.requireNonNull(reporter, "reporter");
        this.enforcer = Objects.requireNonNull(enforcer, "enforcer");
        this.beforeWrite = Objects.requireNonNull(beforeWrite, "beforeWrite");
        this.thread = new Thread(this::run, "athena-replay-writer");
        this.thread.setDaemon(true);
        this.thread.setPriority(Thread.MIN_PRIORITY);
        this.thread.start();
    }

    /**
     * Queues a job. Robot-thread cost is constant: one lock and one queue operation.
     *
     * @param job flush to write
     * @return false when closed or when the job cannot fit beside the current write within the byte budget
     */
    boolean submit(Job job) {
        Objects.requireNonNull(job, "job");
        synchronized (lock) {
            if (closed) {
                return false;
            }
            if (job.retainedBytes() > maxRetainedBytes) {
                dropped++;
                return false;
            }
            // The current job is never cancelled. Drop oldest pending jobs until both limits admit the newest.
            while (queue.remainingCapacity() == 0 || job.retainedBytes() > maxRetainedBytes - retainedBytes) {
                Job oldest = queue.poll();
                if (oldest == null) {
                    // Only the current write holds the remaining budget; reject explicitly instead of blocking.
                    dropped++;
                    return false;
                }
                retainedBytes -= oldest.retainedBytes();
                pending--;
                dropped++;
            }
            queue.add(job);
            retainedBytes += job.retainedBytes();
            pending++;
            return true;
        }
    }

    /** Returns jobs accepted and not yet finished, including the one being written. */
    int pending() {
        synchronized (lock) {
            return pending;
        }
    }

    /** Returns flushes dropped/rejected by the queue or byte budget. */
    long dropped() {
        synchronized (lock) {
            return dropped;
        }
    }

    long retainedBytes() {
        synchronized (lock) {
            return retainedBytes;
        }
    }

    void recordingEvicted(long segments) {
        synchronized (lock) {
            recordingEvictions += segments;
        }
    }

    /** Returns the number of files written. */
    long written() {
        synchronized (lock) {
            return written;
        }
    }

    /** Returns the most recent file written, if any. */
    Optional<Path> lastWritten() {
        return Optional.ofNullable(lastWritten);
    }

    /**
     * Waits until every accepted job has finished.
     *
     * @param timeout maximum wait
     * @return true when idle
     * @throws InterruptedException when interrupted while waiting
     */
    boolean awaitIdle(Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        synchronized (lock) {
            while (pending > 0) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) {
                    return false;
                }
                TimeUnit.NANOSECONDS.timedWait(lock, remaining);
            }
            return true;
        }
    }

    /** Stops accepting jobs. Queued jobs are still written; the thread exits when the queue is empty. */
    void close() {
        synchronized (lock) {
            closed = true;
        }
    }

    private void run() {
        while (true) {
            Job job;
            try {
                job = queue.poll(100, TimeUnit.MILLISECONDS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                return;
            }
            if (job == null) {
                reportDrops();
                synchronized (lock) {
                    if (closed && pending == 0) {
                        return;
                    }
                }
                continue;
            }
            try {
                reportDrops();
                write(job);
            } finally {
                long releasedBytes = job.retainedBytes();
                // Do not advertise free budget while the writer's local still roots the previous large job.
                job = null;
                synchronized (lock) {
                    retainedBytes -= releasedBytes;
                    pending--;
                    lock.notifyAll();
                }
            }
        }
    }

    private void reportDrops() {
        long newlyDropped;
        long newlyEvicted;
        synchronized (lock) {
            newlyDropped = dropped - reportedDropped;
            reportedDropped = dropped;
            newlyEvicted = recordingEvictions - reportedRecordingEvictions;
            reportedRecordingEvictions = recordingEvictions;
        }
        if (newlyDropped > 0) {
            reporter.warning("Athena replay writer queue or memory budget was full; dropped " + newlyDropped
                    + " flush(es) to keep the robot loop on time.", null);
        }
        if (newlyEvicted > 0) {
            reporter.warning("Athena replay disk writer retained the history budget; evicted " + newlyEvicted
                    + " in-RAM segment(s) while keeping the newest recording data.", null);
        }
    }

    private void write(Job job) {
        Path target = job.target();
        try {
            beforeWrite.accept(job);
            ReplayFileWriter.write(target, job.metadata(), job.dictionary(), job.segments());
            lastWritten = target;
            synchronized (lock) {
                written++;
            }
            enforcer.enforce(target.getFileName().toString());
        } catch (IOException | RuntimeException exception) {
            reporter.error("Athena replay flush to " + target + " failed (" + job.reason() + ").", exception);
        }
    }
}
