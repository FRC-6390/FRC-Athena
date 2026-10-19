package ca.frc6390.athena.robot;

import edu.wpi.first.units.measure.Time;
import java.util.Objects;

import static edu.wpi.first.units.Units.Seconds;

/**
 * A small periodic task owned by the root robot runtime.
 */
public record RuntimeWorker(String name, Time period, Runnable task) {
    public RuntimeWorker {
        name = name == null || name.isBlank() ? "worker" : name.trim();
        double seconds = Objects.requireNonNull(period, "period").in(Seconds);
        if (!Double.isFinite(seconds) || seconds <= 0.0) {
            throw new IllegalArgumentException("period must be finite and positive");
        }
        Objects.requireNonNull(task, "task");
    }

    /**
     * Creates a periodic worker task.
     *
     * @param name worker name
     * @param period period
     * @param task task to run
     * @return worker
     */
    public static RuntimeWorker every(String name, Time period, Runnable task) {
        return new RuntimeWorker(name, period, task);
    }
}
