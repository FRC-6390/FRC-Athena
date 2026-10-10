package ca.frc6390.athena.arcp.v2;

/**
 * Subscription rate: either on change or a maximum Hz.
 *
 * @param onChange true to send only when the value changes
 * @param hz maximum update rate when {@code onChange} is false
 */
public record ArcpRate(boolean onChange, double hz) {

    public static final ArcpRate ON_CHANGE = new ArcpRate(true, 0.0);

    public static ArcpRate hz(double hz) {
        return new ArcpRate(false, hz);
    }

    /** @return null when valid, otherwise the reason it is invalid */
    public String validationError() {
        if (onChange) {
            return null;
        }
        if (!(hz > 0.0) || Double.isInfinite(hz) || hz > 1000.0) {
            return "rate must be a finite value in (0, 1000] Hz";
        }
        return null;
    }
}
