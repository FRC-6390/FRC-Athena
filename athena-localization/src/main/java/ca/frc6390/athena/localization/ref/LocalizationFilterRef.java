package ca.frc6390.athena.localization.ref;

/**
 * Filter for localization measurements and intermediate results.
 */
@FunctionalInterface
public interface LocalizationFilterRef {
    /**
     * Returns true when the candidate should be accepted.
     *
     * @param context filter context
     * @return true to accept
     */
    boolean accept(LocalizationFilterContext context);
}
