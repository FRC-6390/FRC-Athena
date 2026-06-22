package ca.frc6390.athena.api.hardware;

/**
 * Stable Athena key for a hardware family or model.
 *
 * <p>Hardware kinds are safe to reference from generic Athena code even when
 * the matching vendor backend is not installed. Validation reports the missing
 * backend later if a robot actually tries to construct that device.</p>
 */
public interface HardwareKind {
    /**
     * Returns the stable namespaced key, such as {@code ctre:talon-fx}.
     *
     * @return hardware key
     */
    String key();
}
