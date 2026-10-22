package ca.frc6390.athena.arcp;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.time.Duration;
import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.Map;

/** One bounded ordered mailbox for every telemetry operation. No producer waits for its consumer. */
final class CyclePublisher {
    static final int CAPACITY = 8;
    static final int MAX_BYTES = 4 * 1024 * 1024;
    private static final int MAX_POOL_BYTES = 1024 * 1024;
    @FunctionalInterface interface Sink { void publish(long handle, long timestamp, ByteBuffer data, int length); }
    @FunctionalInterface interface BufferSink { void publish(ByteBuffer data, int length); }
    private final Sink sink;
    private final Object lock = new Object();
    private final ArrayDeque<Task> queue = new ArrayDeque<>();
    private final ArrayDeque<ByteBuffer> pool = new ArrayDeque<>();
    private final Thread worker;
    private int retainedBytes;
    private int pooledBytes;
    private boolean active;
    private boolean closed;
    private RuntimeException failure;
    private long discardedOperations;

    CyclePublisher(Sink sink) {
        this.sink = sink;
        worker = new Thread(this::run, "athena-arcp-publisher");
        worker.setDaemon(true);
        worker.setPriority(Thread.MIN_PRIORITY);
        worker.start();
    }

    void publish(long handle, long timestamp, ByteBuffer source, int length) {
        ByteBuffer copy = copy(source, length);
        Task task = new Task(handle, timestamp, copy, length, null, null);
        synchronized (lock) {
            checkOpen();
            Task tail = queue.peekLast();
            if (queue.size() >= CAPACITY && tail != null && tail.cycle() && tail.handle == handle) {
                Map<Long, Object> values = new LinkedHashMap<>();
                readValues(tail.data, values);
                readValues(copy, values);
                if (length > MAX_BYTES - tail.length) {
                    recycle(copy);
                    reject("ARCP merged cycle exceeds its transient byte budget");
                }
                ArcpMarshal.Out merged = new ArcpMarshal.Out(take(tail.length + length));
                merged.u32(values.size());
                values.forEach((id, value) -> { merged.u64(id); merged.value(value); });
                int newBytes = retainedBytes - tail.length + merged.length();
                if (newBytes > MAX_BYTES) reject("ARCP publisher byte budget exhausted");
                retainedBytes = newBytes;
                recycle(tail.data);
                recycle(copy);
                tail.data = merged.buffer();
                tail.length = merged.length();
                tail.timestamp = timestamp;
                return;
            }
            admit(task, false);
        }
    }

    /** Copies mutable producer bytes before returning, ordered with cycles and markers. */
    void execute(ByteBuffer source, int length, BufferSink operation) {
        ByteBuffer copy = copy(source, length);
        synchronized (lock) { checkOpen(); admit(new Task(0, 0, copy, length, operation, null), true); }
    }

    void execute(Runnable operation, int retainedBytes) {
        synchronized (lock) { checkOpen(); admit(new Task(0, 0, null, retainedBytes, null, operation), true); }
    }

    /** Best-effort background status sampling; unlike protocol operations, it may simply be retried later. */
    boolean tryExecute(Runnable operation) {
        synchronized (lock) {
            if (closed || failure != null || queue.size() >= CAPACITY - 1) return false;
            queue.addLast(new Task(0, 0, null, 0, null, operation));
            lock.notifyAll();
            return true;
        }
    }

    String failureMessage() {
        synchronized (lock) {
            return failure == null ? "" : failure.getMessage() + "; discarded operations=" + discardedOperations;
        }
    }

    private void admit(Task task, boolean critical) {
        // Critical operations reserve one tail slot so the next cycle always has somewhere to merge.
        if (queue.size() >= (critical ? CAPACITY - 1 : CAPACITY)
                || task.length > MAX_BYTES - retainedBytes) {
            recycle(task.data);
            reject("ARCP ordered publisher full; publication stopped to preserve schema/marker order");
        }
        queue.addLast(task);
        retainedBytes += task.length;
        lock.notifyAll();
    }

    private void reject(String message) {
        failure = new IllegalStateException(message);
        throw failure;
    }

    boolean idle() { synchronized (lock) { return queue.isEmpty() && !active; } }

    /** Explicit test/shutdown fence. Never used by robot publication methods. */
    void awaitIdle() {
        try {
            if (!awaitIdle(Duration.ofSeconds(5))) throw new IllegalStateException("ARCP publisher did not drain");
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted waiting for ARCP publisher", interrupted);
        }
    }

    private boolean awaitIdle(Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        synchronized (lock) {
            while (active || !queue.isEmpty()) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) return false;
                java.util.concurrent.TimeUnit.NANOSECONDS.timedWait(lock, remaining);
            }
            if (failure != null) throw failure;
            return true;
        }
    }

    boolean close() {
        synchronized (lock) { closed = true; lock.notifyAll(); }
        try { worker.join(5000); return !worker.isAlive(); }
        catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); return false; }
    }

    private void checkOpen() {
        if (failure != null) throw failure;
        if (closed) throw new IllegalStateException("ARCP publisher closed");
    }

    private ByteBuffer copy(ByteBuffer source, int length) {
        ByteBuffer copy;
        synchronized (lock) {
            checkOpen();
            if (length < 0 || length > MAX_BYTES) reject("ARCP publication exceeds byte budget: " + length);
            copy = take(length);
        }
        copy.put(0, source, 0, length);
        return copy;
    }

    private ByteBuffer take(int length) {
        ByteBuffer buffer = pool.pollFirst();
        if (buffer != null) pooledBytes -= buffer.capacity();
        if (buffer == null || buffer.capacity() < length) {
            buffer = ByteBuffer.allocateDirect(Math.max(length, ArcpMarshal.INITIAL_CAPACITY))
                    .order(ByteOrder.LITTLE_ENDIAN);
        }
        return buffer.clear();
    }

    private void recycle(ByteBuffer buffer) {
        if (buffer != null && buffer.capacity() <= MAX_POOL_BYTES - pooledBytes) {
            pool.addLast(buffer); pooledBytes += buffer.capacity();
        }
    }

    private void run() {
        while (true) {
            Task task;
            synchronized (lock) {
                while (queue.isEmpty() && !closed) {
                    try { lock.wait(); }
                    catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); return; }
                }
                if (queue.isEmpty()) return;
                task = queue.removeFirst();
                active = true;
            }
            try {
                if (task.cycle()) sink.publish(task.handle, task.timestamp, task.data, task.length);
                else if (task.bufferSink != null) task.bufferSink.publish(task.data, task.length);
                else task.operation.run();
            } catch (Throwable problem) {
                synchronized (lock) {
                    if (failure == null) failure = new IllegalStateException("ARCP publisher failed", problem);
                    // A failed schema/marker cannot be skipped while later accepted frames continue.
                    Task abandoned;
                    while ((abandoned = queue.pollFirst()) != null) {
                        retainedBytes -= abandoned.length;
                        recycle(abandoned.data);
                        discardedOperations++;
                    }
                }
            } finally {
                synchronized (lock) {
                    retainedBytes -= task.length;
                    recycle(task.data);
                    active = false;
                    lock.notifyAll();
                }
            }
        }
    }

    private static final class Task {
        final long handle;
        long timestamp;
        ByteBuffer data;
        int length;
        final BufferSink bufferSink;
        final Runnable operation;
        Task(long handle, long timestamp, ByteBuffer data, int length, BufferSink bufferSink, Runnable operation) {
            this.handle = handle; this.timestamp = timestamp; this.data = data; this.length = length;
            this.bufferSink = bufferSink; this.operation = operation;
        }
        boolean cycle() { return bufferSink == null && operation == null; }
    }

    private static void readValues(ByteBuffer data, Map<Long, Object> values) {
        ArcpMarshal.In in = new ArcpMarshal.In(data);
        int count = in.u32();
        for (int i = 0; i < count; i++) values.put(in.u64(), in.value());
    }
}
