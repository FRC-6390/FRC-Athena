package ca.frc6390.athena.robot;

import java.util.Objects;

/**
 * A small periodic task owned by the root robot runtime.
 */
public record RuntimeWorker(String name, double periodSeconds, Runnable task) {
    public RuntimeWorker {
        name = name == null || name.isBlank() ? "worker" : name.trim();
        if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
            throw new IllegalArgumentException("periodSeconds must be finite and positive");
        }
        Objects.requireNonNull(task, "task");
    }

    /**
     * Creates a periodic worker task.
     *
     * @param name worker name
     * @param periodSeconds period in seconds
     * @param task task to run
     * @return worker
     */
    public static RuntimeWorker every(String name, double periodSeconds, Runnable task) {
        return new RuntimeWorker(name, periodSeconds, task);
    }
}
