package ca.frc6390.athena.hardware.backend;

/**
 * Skips repeated identical open-loop commands to a vendor motor controller.
 *
 * <p>Each vendor set call is a JNI call (and often a CAN frame) on the roboRIO, and most motors receive the same
 * percent or voltage every cycle (neutral, held percent outputs). Both CTRE Phoenix 6 and REVLib keep re-sending
 * the last control frame from their own native threads, so sending the same value again changes nothing on the
 * device. A handle asks {@link #percent}/{@link #voltage} before sending and calls {@link #invalidate()} on every
 * other command or configuration change, so the next open-loop call always goes out. A repeated value is still
 * re-sent every {@value #MAX_SKIPPED} calls as a safety net.
 */
public final class OpenLoopCommandFilter {
    /** Repeats skipped before the same value is sent again anyway (one second at 50 Hz). */
    public static final int MAX_SKIPPED = 50;

    private static final int NONE = 0;
    private static final int PERCENT = 1;
    private static final int VOLTAGE = 2;

    private int mode = NONE;
    private long lastBits;
    private int skipped;

    /**
     * Returns whether a percent command must be sent; records it as sent when it returns true.
     *
     * @param percent sanitized percent output
     * @return true to send the command
     */
    public boolean percent(double percent) {
        return shouldSend(PERCENT, percent);
    }

    /**
     * Returns whether a voltage command must be sent; records it as sent when it returns true.
     *
     * @param volts sanitized voltage
     * @return true to send the command
     */
    public boolean voltage(double volts) {
        return shouldSend(VOLTAGE, volts);
    }

    /** Forgets the last open-loop command; call after any other command, follow change or configuration. */
    public void invalidate() {
        mode = NONE;
        skipped = 0;
    }

    private boolean shouldSend(int requestedMode, double value) {
        long bits = Double.doubleToLongBits(value);
        if (mode == requestedMode && bits == lastBits && skipped < MAX_SKIPPED) {
            skipped++;
            return false;
        }
        mode = requestedMode;
        lastBits = bits;
        skipped = 0;
        return true;
    }
}
