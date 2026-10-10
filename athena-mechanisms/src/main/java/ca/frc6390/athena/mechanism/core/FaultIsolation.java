package ca.frc6390.athena.mechanism.core;

/**
 * Marks exceptions raised under {@link ca.frc6390.athena.api.FailurePolicy#PANIC} so per-mechanism
 * fault isolation lets them stop the robot program as declared instead of containing them.
 */
final class FaultIsolation {
    private static final Throwable PANIC_MARKER = new Throwable("Athena PANIC failure policy", null, false, false) {
    };

    private FaultIsolation() {
    }

    /** Marks an exception that must propagate through fault isolation and returns it. */
    static RuntimeException panic(RuntimeException exception) {
        if (!isPanic(exception)) exception.addSuppressed(PANIC_MARKER);
        return exception;
    }

    private static final long WARNING_PERIOD_NANOS = 5_000_000_000L;
    private static final int WARNING_LIMIT = 256;
    private static final java.util.Map<Object, Long> lastWarnings = new java.util.IdentityHashMap<>();
    private static final java.util.List<String> pendingWarnings = new java.util.ArrayList<>();

    /**
     * Queues a runtime warning for a source (an action or supplier) at most once every five
     * seconds. Used for values that are skipped or held instead of throwing in the robot loop.
     */
    static synchronized void warn(Object source, String message) {
        long now = System.nanoTime();
        Long last = lastWarnings.get(source);
        if (last != null && now - last < WARNING_PERIOD_NANOS) return;
        if (lastWarnings.size() >= WARNING_LIMIT) lastWarnings.clear();
        lastWarnings.put(source, now);
        if (pendingWarnings.size() >= WARNING_LIMIT) pendingWarnings.remove(0);
        pendingWarnings.add(message);
    }

    private static final java.util.List<String> pendingReports = new java.util.ArrayList<>();

    /**
     * Queues a deliberate fault reported by {@code Actions.fault(...)}. Reports are delivered through the
     * runtime fault channel (the failure reporter) by the scheduler, and are never rate limited away.
     */
    static synchronized void report(String message) {
        if (pendingReports.size() >= WARNING_LIMIT) pendingReports.remove(0);
        pendingReports.add(message);
    }

    static synchronized java.util.List<String> drainReports() {
        if (pendingReports.isEmpty()) return java.util.List.of();
        java.util.List<String> drained = java.util.List.copyOf(pendingReports);
        pendingReports.clear();
        return drained;
    }

    static synchronized java.util.List<String> drainWarnings() {
        if (pendingWarnings.isEmpty()) return java.util.List.of();
        java.util.List<String> drained = java.util.List.copyOf(pendingWarnings);
        pendingWarnings.clear();
        return drained;
    }

    static boolean isPanic(Throwable exception) {
        for (Throwable current = exception; current != null; current = current.getCause()) {
            for (Throwable suppressed : current.getSuppressed()) {
                if (suppressed == PANIC_MARKER) return true;
            }
            if (current.getCause() == current) break;
        }
        return false;
    }
}
