package ca.frc6390.athena.vendor.ctre;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Connection check from a signal the handle already refreshes every cycle.
 *
 * <p>Phoenix's {@code ParentDevice.isConnected()} refreshes the Version signal and reads the current time, two JNI
 * calls per device per cycle. This applies the same rule (connected when the device sent a frame within the last
 * 0.5 s) to the system timestamp of a signal in the handle's batched {@code refreshAll}, so it costs no JNI call.
 * A device that has never sent a frame is disconnected. Call {@link #connected()} after the batch refresh.
 */
final class SignalFreshness {
    /** Same threshold as {@code ParentDevice.isConnected()}. */
    static final long MAX_LATENCY_NANOS = 500_000_000L;

    private final BaseStatusSignal signal;
    private double lastFrameSeconds = Double.NaN;
    private long lastFrameNanos;

    SignalFreshness(BaseStatusSignal signal) {
        this.signal = signal;
    }

    boolean connected() {
        var timestamp = signal.getAllTimestamps().getSystemTimestamp();
        if (!timestamp.isValid()) {
            return false;
        }
        long now = System.nanoTime();
        double frameSeconds = timestamp.getTime();
        if (frameSeconds != lastFrameSeconds) {
            lastFrameSeconds = frameSeconds;
            lastFrameNanos = now;
            return true;
        }
        return now - lastFrameNanos <= MAX_LATENCY_NANOS;
    }
}
