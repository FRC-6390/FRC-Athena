package ca.frc6390.athena.hardware.sensor;

/**
 * Direction a limit switch should block.
 */
public enum BlockDirection {
    /** Do not block motion from this sensor. */
    NONE(0),

    /** Block negative motion. */
    NEGATIVE(-1),

    /** Block positive motion. */
    POSITIVE(1);

    private final int multiplier;

    BlockDirection(int multiplier) {
        this.multiplier = multiplier;
    }

    /**
     * Returns the direction multiplier used by motion code.
     *
     * @return -1, 0, or 1
     */
    public int multiplier() {
        return multiplier;
    }
}
