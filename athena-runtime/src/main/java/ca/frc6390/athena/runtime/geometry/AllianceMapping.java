package ca.frc6390.athena.runtime.geometry;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * Internal, unit-free alliance mapping used by the WPILib facade
 * ({@code ca.frc6390.athena.wpilib.geometry.AllianceTransform}), which is the team-facing type.
 *
 * <p>Robot code should not use this class. It works in primitive meters and radians on
 * {@link PoseSnapshot}, and the WPILib facade converts {@code Pose2d}, {@code Distance} and
 * {@code Rotation2d} at its boundary. It is named {@code AllianceMapping} so that it cannot be
 * confused with the team-facing {@code AllianceTransform}.
 *
 * <p>Authoring is done once, in blue-alliance coordinates. Each call resolves the current alliance
 * through the alliance source: blue (and unknown) returns the input unchanged, and red returns the
 * transformed value. Headings are radians in {@code (-pi, pi]}.
 */
public final class AllianceMapping {
    private static final BooleanSupplier BLUE = () -> false;
    private static final UnaryOperator<PoseSnapshot> IDENTITY = pose -> pose;

    private final UnaryOperator<PoseSnapshot> redMapping;
    private final BooleanSupplier isRed;

    private AllianceMapping(UnaryOperator<PoseSnapshot> redMapping, BooleanSupplier isRed) {
        this.redMapping = Objects.requireNonNull(redMapping, "redMapping");
        this.isRed = Objects.requireNonNull(isRed, "isRed");
    }

    /** Returns a mapping that never changes geometry. */
    public static AllianceMapping identity() {
        return new AllianceMapping(IDENTITY, BLUE);
    }

    /** Returns a mapping that applies {@code redMapping} on the red alliance. */
    public static AllianceMapping of(UnaryOperator<PoseSnapshot> redMapping) {
        return new AllianceMapping(Objects.requireNonNull(redMapping, "redMapping"), BLUE);
    }

    /** Point-symmetric mapping about the field center; headings turn by 180 degrees. */
    public static AllianceMapping rotate180(double fieldLengthMeters, double fieldWidthMeters) {
        requireFinite(fieldLengthMeters, "fieldLength");
        requireFinite(fieldWidthMeters, "fieldWidth");
        return new AllianceMapping(pose -> new PoseSnapshot(
                fieldLengthMeters - pose.xMeters(),
                fieldWidthMeters - pose.yMeters(),
                normalize(pose.headingRadians() + Math.PI)), BLUE);
    }

    /** Mirror across the field centerline; headings reflect as {@code pi - heading}. */
    public static AllianceMapping mirrorX(double fieldLengthMeters) {
        requireFinite(fieldLengthMeters, "fieldLength");
        return new AllianceMapping(pose -> new PoseSnapshot(
                fieldLengthMeters - pose.xMeters(),
                pose.yMeters(),
                normalize(Math.PI - pose.headingRadians())), BLUE);
    }

    /** Mirror across the field width; headings reflect as {@code -heading}. */
    public static AllianceMapping mirrorY(double fieldWidthMeters) {
        requireFinite(fieldWidthMeters, "fieldWidth");
        return new AllianceMapping(pose -> new PoseSnapshot(
                pose.xMeters(),
                fieldWidthMeters - pose.yMeters(),
                normalize(-pose.headingRadians())), BLUE);
    }

    /** Returns the same mapping resolved against {@code isRed}. A {@code null} source is blue. */
    public AllianceMapping withAlliance(BooleanSupplier isRed) {
        return new AllianceMapping(redMapping, isRed == null ? BLUE : isRed);
    }

    /** Returns the pose for the current alliance. */
    public PoseSnapshot pose(PoseSnapshot bluePose) {
        Objects.requireNonNull(bluePose, "bluePose");
        return isRed.getAsBoolean() ? redMapping.apply(bluePose) : bluePose;
    }

    /** Returns the translation for the current alliance. */
    public Translation2d translation(Translation2d blueTranslation) {
        Objects.requireNonNull(blueTranslation, "blueTranslation");
        if (!isRed.getAsBoolean()) {
            return blueTranslation;
        }
        return translationForRed(blueTranslation.getX(), blueTranslation.getY());
    }

    /** Returns the heading in radians for the current alliance. */
    public double rotation(double blueHeadingRadians) {
        requireFinite(blueHeadingRadians, "blueHeadingRadians");
        if (!isRed.getAsBoolean()) {
            return blueHeadingRadians;
        }
        return redMapping.apply(new PoseSnapshot(0.0, 0.0, blueHeadingRadians)).headingRadians();
    }

    /** Returns the rectangle for the current alliance as the bounding box of its mapped corners. */
    public Rectangle2d region(Rectangle2d blueRegion) {
        Objects.requireNonNull(blueRegion, "blueRegion");
        if (!isRed.getAsBoolean()) {
            return blueRegion;
        }
        Translation2d[] corners = {
            translationForRed(blueRegion.minimumXMeters(), blueRegion.minimumYMeters()),
            translationForRed(blueRegion.maximumXMeters(), blueRegion.minimumYMeters()),
            translationForRed(blueRegion.minimumXMeters(), blueRegion.maximumYMeters()),
            translationForRed(blueRegion.maximumXMeters(), blueRegion.maximumYMeters())
        };
        double minimumX = Double.POSITIVE_INFINITY;
        double minimumY = Double.POSITIVE_INFINITY;
        double maximumX = Double.NEGATIVE_INFINITY;
        double maximumY = Double.NEGATIVE_INFINITY;
        for (Translation2d corner : corners) {
            minimumX = Math.min(minimumX, corner.getX());
            minimumY = Math.min(minimumY, corner.getY());
            maximumX = Math.max(maximumX, corner.getX());
            maximumY = Math.max(maximumY, corner.getY());
        }
        return Rectangle2d.of(
                edu.wpi.first.units.Units.Meters.of(minimumX),
                edu.wpi.first.units.Units.Meters.of(minimumY),
                edu.wpi.first.units.Units.Meters.of(maximumX),
                edu.wpi.first.units.Units.Meters.of(maximumY));
    }

    /** Returns a supplier that resolves the alliance on every call. */
    public Supplier<PoseSnapshot> poseSupplier(PoseSnapshot bluePose) {
        Objects.requireNonNull(bluePose, "bluePose");
        return () -> pose(bluePose);
    }

    private Translation2d translationForRed(double x, double y) {
        PoseSnapshot mapped = redMapping.apply(new PoseSnapshot(x, y, 0.0));
        return new Translation2d(mapped.xMeters(), mapped.yMeters());
    }

    /** Wraps a heading into {@code (-pi, pi]}. */
    static double normalize(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite.");
        }
    }
}
