package ca.frc6390.athena.runtime.filter;

/**
 * Dependency-free 2D pose snapshot.
 *
 * @param xMeters x position
 * @param yMeters y position
 * @param headingRadians heading
 */
public record PoseSnapshot(double xMeters, double yMeters, double headingRadians) {
}
