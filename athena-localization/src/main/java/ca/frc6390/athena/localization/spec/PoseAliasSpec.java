package ca.frc6390.athena.localization.spec;

import java.util.Objects;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Named autonomous pose target.
 *
 * @param name alias name
 * @param pose field-relative pose
 */
public record PoseAliasSpec(String name, PoseSnapshot pose) {
    public PoseAliasSpec {
        name = name == null || name.isBlank() ? "pose" : name;
        pose = Objects.requireNonNull(pose, "pose");
    }

    /**
     * Returns true when all pose values are finite.
     *
     * @return true if finite
     */
    public boolean isFinite() {
        return Double.isFinite(pose.xMeters())
                && Double.isFinite(pose.yMeters())
                && Double.isFinite(pose.headingRadians());
    }
}
