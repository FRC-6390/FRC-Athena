package ca.frc6390.athena.mechanism.model;

import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import java.util.ArrayDeque;
import java.util.concurrent.Executor;
import java.util.concurrent.RejectedExecutionException;
import java.util.function.Supplier;

/** Bounded serial export mailbox. Providers are never opened or called under a robot-visible monitor. */
final class AsyncSysIdLog implements SysIdLog {
    static final int SAMPLE_CAPACITY = 256;
    static final int END_CAPACITY = 16;
    private record Event(SysIdSample sample) { }
    private final ArrayDeque<Event> queue = new ArrayDeque<>();
    private final Supplier<SysIdLog> log;
    private final Executor executor;
    private int samples;
    private int ends;
    private boolean scheduled;
    private volatile long dropped;
    private volatile String failure = "";

    AsyncSysIdLog(Supplier<SysIdLog> log) { this(log, ModelBackground.EXPORTS); }
    AsyncSysIdLog(Supplier<SysIdLog> log, Executor executor) { this.log = log; this.executor = executor; }

    @Override public void record(SysIdSample sample) {
        synchronized (queue) {
            if (samples == SAMPLE_CAPACITY) {
                var iterator = queue.iterator();
                while (iterator.hasNext()) {
                    if (iterator.next().sample() != null) { iterator.remove(); samples--; dropped++; break; }
                }
            }
            queue.addLast(new Event(sample));
            samples++;
            schedule();
        }
    }

    @Override public void end() {
        synchronized (queue) {
            if (ends == END_CAPACITY) {
                failure = "SysId export end queue full; end refused";
                return;
            }
            queue.addLast(new Event(null)); ends++;
            schedule();
        }
    }

    long dropped() { return dropped; }
    String failure() { return failure; }
    int pending() { synchronized (queue) { return queue.size(); } }

    /** Explicit test/shutdown wait. No routine callback calls this method. */
    boolean awaitIdle(java.time.Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        synchronized (queue) {
            while (scheduled || !queue.isEmpty()) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) return false;
                java.util.concurrent.TimeUnit.NANOSECONDS.timedWait(queue, remaining);
            }
            return true;
        }
    }

    private void schedule() {
        if (scheduled) return;
        scheduled = true;
        try { executor.execute(this::drain); }
        catch (RejectedExecutionException rejected) {
            failure = "SysId shared export worker full; queued export refused";
            dropped += samples;
            queue.clear(); samples = 0; ends = 0; scheduled = false; queue.notifyAll();
        }
    }

    private void drain() {
        for (int processed = 0; processed < 32; processed++) {
            Event event;
            synchronized (queue) {
                event = queue.pollFirst();
                if (event == null) { scheduled = false; queue.notifyAll(); return; }
                if (event.sample() == null) ends--; else samples--;
            }
            try {
                SysIdLog target = log.get();
                if (target != null) {
                    if (event.sample() == null) target.end(); else target.record(event.sample());
                }
            } catch (Throwable problem) {
                failure = "SysId export failed: " + problem.getMessage();
            }
        }
        synchronized (queue) {
            scheduled = false;
            if (!queue.isEmpty()) schedule();
            else queue.notifyAll();
        }
    }
}
