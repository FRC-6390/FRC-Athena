package ca.frc6390.athena.hardware.backend;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Bounded per-controller configuration mailbox, serialized on four shared daemon workers. Different
 * configuration kinds retain order; a newer queued value replaces a superseded value of the same kind.
 * The loop only submits requests and reads acknowledgements. A blocked controller never owns a graph lock.
 */
public final class ConfigurationQueue implements AutoCloseable {
    public enum Key { ACTIVATION, RUNTIME, SLOT, FOLLOW, POSITION, INTEGRATED_ENCODER, ABSOLUTE_ENCODER }

    private static final AtomicInteger THREAD_IDS = new AtomicInteger();
    private static final ThreadPoolExecutor WORKERS = workers();
    private static ThreadPoolExecutor workers() {
        ThreadPoolExecutor pool = new ThreadPoolExecutor(4, 4, 10, TimeUnit.SECONDS,
                new ArrayBlockingQueue<>(256), runnable -> {
                    Thread thread = new Thread(runnable, "athena-hardware-config-" + THREAD_IDS.incrementAndGet());
                    thread.setDaemon(true);
                    return thread;
                });
        pool.allowCoreThreadTimeOut(true);
        return pool;
    }

    private final String name;
    private final ThreadPoolExecutor executor;
    private final Runnable stop;
    private final Map<Key, Request> desired = new EnumMap<>(Key.class);
    private final LinkedHashMap<Key, Request> queued = new LinkedHashMap<>();
    private volatile ConfigurationStatus status = ConfigurationStatus.READY;
    private long generation;
    private boolean running;
    private boolean closed;
    private Future<?> task;
    private Thread workerThread;
    private Runnable cleanup = () -> { };

    public ConfigurationQueue(String name, Runnable stop) {
        this(name, stop, WORKERS);
    }

    ConfigurationQueue(String name, Runnable stop, ThreadPoolExecutor executor) {
        this.name = Objects.requireNonNull(name, "name");
        this.stop = Objects.requireNonNull(stop, "stop");
        this.executor = Objects.requireNonNull(executor, "executor");
    }

    public ConfigurationStatus status() { return status; }

    /** Returns true only when this exact request is acknowledged and all required work is ready. */
    public boolean request(Key key, Object value, Runnable operation) {
        boolean mustStop;
        synchronized (this) {
            if (closed) throw new IllegalStateException("Configuration is closed for " + name);
            Request previous = desired.get(key);
            if (previous != null && Objects.equals(previous.value, value)
                    && !(key == Key.POSITION && previous.applied)) {
                retryFailed();
                return previous.applied && status.ready();
            }
            mustStop = status.ready();
            Request request = new Request(key, value, operation, ++generation);
            desired.put(key, request);
            queued.remove(key);
            queued.put(key, request);
            publish();
        }
        // Neutral is a nonblocking control request, never a configurator/wait operation. It happens
        // before the worker starts so old output cannot remain active during required configuration.
        try {
            if (mustStop) stop.run();
        } catch (RuntimeException exception) {
            // A disconnected device may reject neutral. Required configuration remains gated.
        }
        synchronized (this) { schedule(); }
        return false;
    }

    /** Retries failed latest requests with capped backoff; called by the ordinary input refresh. */
    public synchronized void retryFailed() {
        if (closed || status.ready() || (status.pending() && !status.failed())) return;
        long now = System.nanoTime();
        for (Request request : desired.values()) {
            if (request.error != null && now >= request.retryAt && !queued.containsKey(request.key)) {
                request.error = null;
                queued.put(request.key, request);
            }
        }
        if (queued.size() > 1) {
            var ordered = new java.util.ArrayList<>(queued.values());
            ordered.sort(java.util.Comparator.comparingLong(request -> request.generation));
            queued.clear();
            for (Request request : ordered) queued.put(request.key, request);
        }
        publish();
        schedule();
    }

    private void schedule() {
        if (running || closed || queued.isEmpty()) return;
        long firstGeneration = queued.values().iterator().next().generation;
        for (Request request : desired.values()) {
            if (request.error != null && request.generation < firstGeneration) return;
        }
        running = true;
        try {
            task = executor.submit(this::drain);
        } catch (RuntimeException exception) {
            running = false;
            for (Request request : queued.values()) {
                request.error = new IllegalStateException("Configuration worker queue is full for " + name, exception);
                request.retryAt = System.nanoTime() + TimeUnit.SECONDS.toNanos(1);
            }
            queued.clear();
            publish();
        }
    }

    private void drain() {
        synchronized (this) { workerThread = Thread.currentThread(); }
        try {
            while (true) {
                Request request;
                synchronized (this) {
                    if (closed || queued.isEmpty()) return;
                    request = queued.remove(queued.keySet().iterator().next());
                }
                RuntimeException failure = null;
                try {
                    request.operation.run();
                } catch (RuntimeException | Error exception) {
                    failure = exception instanceof RuntimeException runtime ? runtime
                            : new IllegalStateException("Hardware configuration failed for " + name, exception);
                }
                synchronized (this) {
                    // A cancelled route/value or a closed handle must never acknowledge a newer request.
                    if (!closed && desired.get(request.key) == request) {
                        request.applied = failure == null;
                        request.error = failure;
                        if (failure != null) {
                            request.failures++;
                            request.retryAt = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(
                                    Math.min(2000L, 100L << Math.min(request.failures - 1, 4)));
                        }
                    }
                    publish();
                }
                // Required work after a failure must wait behind its retry/supersession, not overtake it.
                if (failure != null) return;
            }
        } finally {
            Runnable release = null;
            synchronized (this) {
                running = false;
                task = null;
                workerThread = null;
                if (closed) release = cleanup;
                else schedule();
                notifyAll();
            }
            if (release != null) release.run();
        }
    }

    private void publish() {
        boolean pending = false;
        String error = "";
        for (Request request : desired.values()) {
            pending |= !request.applied && request.error == null;
            if (request.error != null) error = request.error.toString();
        }
        if (closed) error = "Closed";
        boolean ready = !pending && error.isEmpty();
        status = new ConfigurationStatus(generation, ready ? generation : status.appliedGeneration(), pending, error);
    }

    /** Waits only in explicit setup/tests; robot periodic code must use {@link #status()}. */
    public synchronized boolean awaitReady(long timeoutMillis) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMillis);
        while (!status.ready()) {
            if (status.failed() || closed) return false;
            long remaining = deadline - System.nanoTime();
            if (remaining <= 0) return false;
            TimeUnit.NANOSECONDS.timedWait(this, remaining);
        }
        return true;
    }

    @Override public void close() { close(() -> { }); }

    /** Fences results immediately; native cleanup runs after any in-flight operation exits. */
    public void close(Runnable release) {
        boolean releaseNow;
        synchronized (this) {
            if (closed) return;
            closed = true;
            cleanup = Objects.requireNonNull(release, "release");
            queued.clear();
            publish();
            // A queued task can be removed and released here. If a worker already took it, let its
            // finally block release resources; cancelling FutureTask before run would skip finally.
            if (task instanceof Runnable runnable && executor.remove(runnable)) {
                running = false;
                task = null;
            }
            releaseNow = !running;
            if (workerThread != null) workerThread.interrupt();
            notifyAll();
        }
        if (releaseNow) release.run();
    }

    private static final class Request {
        final Key key;
        final Object value;
        final Runnable operation;
        final long generation;
        boolean applied;
        RuntimeException error;
        int failures;
        long retryAt;
        Request(Key key, Object value, Runnable operation, long generation) {
            this.key = key;
            this.value = value;
            this.operation = Objects.requireNonNull(operation, "operation");
            this.generation = generation;
        }
    }
}
