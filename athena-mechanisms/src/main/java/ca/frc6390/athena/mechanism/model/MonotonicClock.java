package ca.frc6390.athena.mechanism.model;

import java.util.function.DoubleSupplier;

/** Monotonic seconds source used by checks and characterization. Tests may inject their own. */
public final class MonotonicClock {
    /** Seconds from {@link System#nanoTime()}. */
    public static final DoubleSupplier SYSTEM = () -> System.nanoTime() * 1.0e-9;

    private MonotonicClock() {
    }
}
