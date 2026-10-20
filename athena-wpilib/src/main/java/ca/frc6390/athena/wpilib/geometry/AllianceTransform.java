package ca.frc6390.athena.wpilib.geometry;

import static edu.wpi.first.units.Units.Meters;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.geometry.AllianceMapping;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * Team-facing alliance transform in WPILib types. Authoring is done once in blue-alliance
 * coordinates. Each call resolves the current Driver Station alliance: blue or unknown returns the
 * input unchanged, and red returns the transformed value.
 *
 * <p>This is the only team-facing alliance type. The unit-free engine it delegates to is
 * {@link AllianceMapping} in the runtime package, which is internal.
 */
public final class AllianceTransform {
    private final AllianceMapping delegate;

    private AllianceTransform(AllianceMapping delegate) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
    }

    /** Returns a transform that never changes geometry. */
    public static AllianceTransform identity() {
        return bind(AllianceMapping.identity());
    }

    /** Returns a transform that applies {@code redMapping} on the red alliance. */
    public static AllianceTransform of(UnaryOperator<Pose2d> redMapping) {
        Objects.requireNonNull(redMapping, "redMapping");
        return bind(AllianceMapping.of(pose -> toSnapshot(redMapping.apply(toPose(pose)))));
    }

    /** Returns the point-symmetric transform about the field center. Headings turn by 180 degrees. */
    public static AllianceTransform rotate180(Distance fieldLength, Distance fieldWidth) {
        return bind(AllianceMapping.rotate180(meters(fieldLength), meters(fieldWidth)));
    }

    /** Returns the mirror across the field centerline. Headings reflect as {@code pi - heading}. */
    public static AllianceTransform mirrorX(Distance fieldLength) {
        return bind(AllianceMapping.mirrorX(meters(fieldLength)));
    }

    /** Returns the mirror across the field width. Headings reflect as {@code -heading}. */
    public static AllianceTransform mirrorY(Distance fieldWidth) {
        return bind(AllianceMapping.mirrorY(meters(fieldWidth)));
    }

    /** Returns the same transform resolved against {@code isRed} instead of the Driver Station. */
    public AllianceTransform withAlliance(BooleanSupplier isRed) {
        return new AllianceTransform(delegate.withAlliance(isRed));
    }

    /** Returns whether the Driver Station currently reports the red alliance. Unknown is blue. */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false);
    }

    /** Returns the pose for the current alliance. */
    public Pose2d pose(Pose2d bluePose) {
        Objects.requireNonNull(bluePose, "bluePose");
        return toPose(delegate.pose(toSnapshot(bluePose)));
    }

    /** Returns the translation for the current alliance. */
    public Translation2d translation(Translation2d blueTranslation) {
        return delegate.translation(Objects.requireNonNull(blueTranslation, "blueTranslation"));
    }

    /** Returns the rotation for the current alliance. */
    public Rotation2d rotation(Rotation2d blueRotation) {
        Objects.requireNonNull(blueRotation, "blueRotation");
        return new Rotation2d(delegate.rotation(blueRotation.getRadians()));
    }

    /** Returns the region for the current alliance as the bounding box of its mapped corners. */
    public Rectangle2d region(Rectangle2d blueRegion) {
        return delegate.region(blueRegion);
    }

    /** Returns a supplier that resolves the alliance on every call. */
    public Supplier<Pose2d> poseSupplier(Pose2d bluePose) {
        Objects.requireNonNull(bluePose, "bluePose");
        return () -> pose(bluePose);
    }

    private static AllianceTransform bind(AllianceMapping mapping) {
        return new AllianceTransform(mapping.withAlliance(AllianceTransform::isRedAlliance));
    }

    private static double meters(Distance distance) {
        return Objects.requireNonNull(distance, "distance").in(Meters);
    }

    private static Pose2d toPose(PoseSnapshot pose) {
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    private static PoseSnapshot toSnapshot(Pose2d pose) {
        return new PoseSnapshot(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }
}
