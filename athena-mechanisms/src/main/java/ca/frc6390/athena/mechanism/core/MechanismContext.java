package ca.frc6390.athena.mechanism.core;

/**
 * Runtime facts visible while mechanism states are evaluated.
 */
public record MechanismContext(
        double nowSeconds,
        double timeInStateSeconds,
        double dtSeconds,
        boolean enabled,
        boolean autonomous,
        boolean simulation) {
    public static MechanismContext empty() {
        return new MechanismContext(0.0, 0.0, 0.02, true, false, false);
    }
}
