package ca.frc6390.athena.runtime.geometry;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * Maps blue-alliance field geometry to the red alliance.
 *
 * <p>Authoring is done once, in blue-alliance coordinates. Each call resolves the current alliance
 * through the alliance source: blue (and unknown) returns the input unchanged, and red returns the
 * transformed value. The default source reports blue; robot code binds the Driver Station through
 * the WPILib facade, or injects a source with {@link #withAlliance(BooleanSupplier)} in tests.
 *
 * <p>Headings are radians in {@code (-pi, pi]}. Region transforms map the rectangle's corners and
 * return their bounding box, which is exact for the axis-aligned transforms in this class.
 */
public final class AllianceTransform {
    private static final BooleanSupplier BLUE = () -> false;
    private static final UnaryOperator<PoseSnapshot> IDENTITY = pose -> pose;

    private final UnaryOperator<PoseSnapshot> redMapping;
    private final BooleanSupplier isRed;

    private AllianceTransform(UnaryOperator<PoseSnapshot> redMapping, BooleanSupplier isRed) {
        this.redMapping = Objects.requireNonNull(redMapping, "redMapping");
        this.isRed = Objects.requireNonNull(isRed, "isRed");
    }

    /** Returns a transform that never changes geometry. */
    public static AllianceTransform identity() {
        return new AllianceTransform(IDENTITY, BLUE);
    }

    /**
     * Returns a transform that applies {@code redMapping} on the red alliance.
     *
     * @param redMapping mapping from blue-alliance pose to red-alliance pose
     */
    public static AllianceTransform of(UnaryOperator<PoseSnapshot> redMapping) {
        return new AllianceTransform(Objects.requireNonNull(redMapping, "redMapping"), BLUE);
    }

    /**
     * Returns the transform for a field that is point-symmetric about its center: red positions are
     * {@code (fieldLength - x, fieldWidth - y)} and headings turn by 180 degrees.
     */
    public static AllianceTransform rotate180(double fieldLength, double fieldWidth) {
        requireFinite(fieldLength, "fieldLength");
        requireFinite(fieldWidth, "fieldWidth");
        return new AllianceTransform(pose -> new PoseSnapshot(
                fieldLength - pose.xMeters(),
                fieldWidth - pose.yMeters(),
                normalize(pose.headingRadians() + Math.PI)), BLUE);
    }

    /**
     * Returns the transform that mirrors across the field's centerline: red x becomes
     * {@code fieldLength - x}, y is unchanged, and headings reflect as {@code pi - heading}.
     */
    public static AllianceTransform mirrorX(double fieldLength) {
        requireFinite(fieldLength, "fieldLength");
        return new AllianceTransform(pose -> new PoseSnapshot(
                fieldLength - pose.xMeters(),
                pose.yMeters(),
                normalize(Math.PI - pose.headingRadians())), BLUE);
    }

    /**
     * Returns the transform that mirrors across the field's width: red y becomes
     * {@code fieldWidth - y}, x is unchanged, and headings reflect as {@code -heading}.
     */
    public static AllianceTransform mirrorY(double fieldWidth) {
        requireFinite(fieldWidth, "fieldWidth");
        return new AllianceTransform(pose -> new PoseSnapshot(
                pose.xMeters(),
                fieldWidth - pose.yMeters(),
                normalize(-pose.headingRadians())), BLUE);
    }

    /**
     * Returns the same mapping resolved against {@code isRed}. Use this to inject an alliance source
     * in tests or to supply a robot-specific source. A {@code null} source is treated as blue.
     */
    public AllianceTransform withAlliance(BooleanSupplier isRed) {
        return new AllianceTransform(redMapping, isRed == null ? BLUE : isRed);
    }

    /** Returns the pose for the current alliance. */
    public PoseSnapshot pose(PoseSnapshot bluePose) {
        Objects.requireNonNull(bluePose, "bluePose");
        return isRed.getAsBoolean() ? redMapping.apply(bluePose) : bluePose;
    }

    /** Returns the translation for the current alliance. */
    public Point2d translation(Point2d blueTranslation) {
        Objects.requireNonNull(blueTranslation, "blueTranslation");
        if (!isRed.getAsBoolean()) {
            return blueTranslation;
        }
        PoseSnapshot mapped = redMapping.apply(new PoseSnapshot(blueTranslation.x(), blueTranslation.y(), 0.0));
        return new Point2d(mapped.xMeters(), mapped.yMeters());
    }

    /** Returns the heading in radians for the current alliance. */
    public double rotation(double blueHeadingRadians) {
        requireFinite(blueHeadingRadians, "blueHeadingRadians");
        if (!isRed.getAsBoolean()) {
            return blueHeadingRadians;
        }
        return redMapping.apply(new PoseSnapshot(0.0, 0.0, blueHeadingRadians)).headingRadians();
    }

    /**
     * Returns the rectangle for the current alliance. The result is the bounding box of the mapped
     * corners.
     */
    public Rectangle2d region(Rectangle2d blueRegion) {
        Objects.requireNonNull(blueRegion, "blueRegion");
        if (!isRed.getAsBoolean()) {
            return blueRegion;
        }
        Point2d[] corners = {
            translationForRed(blueRegion.minimumX(), blueRegion.minimumY()),
            translationForRed(blueRegion.maximumX(), blueRegion.minimumY()),
            translationForRed(blueRegion.minimumX(), blueRegion.maximumY()),
            translationForRed(blueRegion.maximumX(), blueRegion.maximumY())
        };
        double minimumX = Double.POSITIVE_INFINITY;
        double minimumY = Double.POSITIVE_INFINITY;
        double maximumX = Double.NEGATIVE_INFINITY;
        double maximumY = Double.NEGATIVE_INFINITY;
        for (Point2d corner : corners) {
            minimumX = Math.min(minimumX, corner.x());
            minimumY = Math.min(minimumY, corner.y());
            maximumX = Math.max(maximumX, corner.x());
            maximumY = Math.max(maximumY, corner.y());
        }
        return new Rectangle2d(minimumX, minimumY, maximumX, maximumY);
    }

    /**
     * Returns a supplier that resolves the alliance on every call, so a value captured at
     * construction time still follows the current alliance.
     */
    public Supplier<PoseSnapshot> poseSupplier(PoseSnapshot bluePose) {
        Objects.requireNonNull(bluePose, "bluePose");
        return () -> pose(bluePose);
    }

    private Point2d translationForRed(double x, double y) {
        PoseSnapshot mapped = redMapping.apply(new PoseSnapshot(x, y, 0.0));
        return new Point2d(mapped.xMeters(), mapped.yMeters());
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
