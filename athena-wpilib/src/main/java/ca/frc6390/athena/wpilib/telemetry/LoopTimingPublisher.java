package ca.frc6390.athena.wpilib.telemetry;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.LongSupplier;

/**
 * Publishes how long the robot loop takes, under {@code /Athena/System/Loop}.
 *
 * <p>Work time runs from the start of the mode periodic callback to the end of {@code robotPeriodic} (Athena's
 * mechanisms, telemetry, replay and ARCP). Period is the time between the starts of consecutive cycles; it grows
 * past 20 ms when the loop overruns. Values are averaged over one-second windows, alongside the window maximum,
 * and published once per window so the measurement itself costs almost nothing.
 *
 * <ul>
 *   <li>{@code WorkMillis}, {@code MaxWorkMillis}: average and worst work time in the last window</li>
 *   <li>{@code PeriodMillis}, {@code MaxPeriodMillis}: average and worst cycle period in the last window</li>
 *   <li>{@code Load}: average work time divided by average period (1.0 means the loop never idles)</li>
 *   <li>{@code Overruns}: cycles since boot whose period exceeded 25 ms</li>
 * </ul>
 */
public final class LoopTimingPublisher implements AutoCloseable {
    public static final String ROOT = SystemTelemetryPublisher.ROOT + "/Loop";
    private static final long WINDOW_NANOS = 1_000_000_000L;
    private static final long OVERRUN_NANOS = 25_000_000L;

    private final LongSupplier clock;
    private final NetworkTableEntry work;
    private final NetworkTableEntry maxWork;
    private final NetworkTableEntry period;
    private final NetworkTableEntry maxPeriod;
    private final NetworkTableEntry load;
    private final NetworkTableEntry overruns;
    private long cycleStart = -1;
    private long windowStart = -1;
    private long windowWork;
    private long windowMaxWork;
    private long windowPeriod;
    private long windowMaxPeriod;
    private int windowCycles;
    private int windowPeriods;
    private long overrunCount;

    public LoopTimingPublisher() {
        this(NetworkTableInstance.getDefault(), System::nanoTime);
    }

    LoopTimingPublisher(NetworkTableInstance instance, LongSupplier clock) {
        this.clock = clock;
        work = instance.getEntry(ROOT + "/WorkMillis");
        maxWork = instance.getEntry(ROOT + "/MaxWorkMillis");
        period = instance.getEntry(ROOT + "/PeriodMillis");
        maxPeriod = instance.getEntry(ROOT + "/MaxPeriodMillis");
        load = instance.getEntry(ROOT + "/Load");
        overruns = instance.getEntry(ROOT + "/Overruns");
    }

    /** Marks the start of a cycle's work (the mode periodic callback). */
    public void cycleStarted() {
        long now = clock.getAsLong();
        if (cycleStart >= 0) {
            long elapsed = now - cycleStart;
            windowPeriod += elapsed;
            windowMaxPeriod = Math.max(windowMaxPeriod, elapsed);
            windowPeriods++;
            if (elapsed > OVERRUN_NANOS) overrunCount++;
        }
        if (windowStart < 0) windowStart = now;
        cycleStart = now;
    }

    /** Marks the end of a cycle's work (end of {@code robotPeriodic}) and publishes once per window. */
    public void cycleFinished() {
        if (cycleStart < 0) return;
        long now = clock.getAsLong();
        long elapsed = now - cycleStart;
        windowWork += elapsed;
        windowMaxWork = Math.max(windowMaxWork, elapsed);
        windowCycles++;
        if (now - windowStart < WINDOW_NANOS) return;
        double averageWork = windowWork / (double) windowCycles;
        double averagePeriod = windowPeriods == 0 ? Double.NaN : windowPeriod / (double) windowPeriods;
        work.setDouble(averageWork / 1.0e6);
        maxWork.setDouble(windowMaxWork / 1.0e6);
        period.setDouble(averagePeriod / 1.0e6);
        maxPeriod.setDouble(windowMaxPeriod / 1.0e6);
        load.setDouble(averageWork / averagePeriod);
        overruns.setInteger(overrunCount);
        windowStart = now;
        windowWork = 0;
        windowMaxWork = 0;
        windowPeriod = 0;
        windowMaxPeriod = 0;
        windowCycles = 0;
        windowPeriods = 0;
    }

    @Override
    public void close() {
        work.close();
        maxWork.close();
        period.close();
        maxPeriod.close();
        load.close();
        overruns.close();
    }
}
