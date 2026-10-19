package ca.frc6390.athena.drivetrain;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Driver-oriented heading for field-oriented teleop, derived from the localized field pose.
 *
 * <p>Field geometry (Choreo paths, localization, {@code AllianceTransform})
 * always has +X pointing away from the <em>blue</em> driver station. A driver's forward points away from
 * their <em>own</em> driver station: field +X on blue, 180 degrees away on red. This type owns that offset, so
 * the robot never re-aligns a separate yaw by hand:</p>
 *
 * <pre>
 *   forward      = allianceForward + driverOffset      (blue 0, red pi, plus the driver's reset offset)
 *   driverHeading = normalize(fieldHeading - forward)  (0 when the robot faces the driver's forward)
 * </pre>
 *
 * <p>Pass {@link #headingRadians()} to {@code SwerveKinematics.drive(pool, heading)} (or
 * {@code SwerveKinematics.drive(pool, orientation)}). Stick input stays in the driver frame, so forward on
 * either alliance drives away from the driver's own station.</p>
 *
 * <p>Behaviour:</p>
 * <ul>
 *   <li><b>Auto to teleop.</b> Nothing to re-align. Auto ends with the localized field pose, which already
 *       carries the heading, so teleop reads the same field heading and the alliance offset.</li>
 *   <li><b>Unknown alliance.</b> Until the Driver Station reports an alliance, forward is blue. The value
 *       updates on the next cycle when the alliance arrives. The Driver Station reports it before enable in
 *       competition. {@link #allianceKnown()} reports whether it is known.</li>
 *   <li><b>No auto and no seed.</b> {@link #assumeDriverForward()} sets the pose heading to the alliance forward
 *       (robot facing away from its own wall) only while {@code poseSeeded} is false. A seeded pose (auto or
 *       vision) is left alone. Bind it to teleop start.</li>
 *   <li><b>Driver reset (Start).</b> {@link #resetHeading()} makes the current robot facing the driver's forward.
 *       It stores an offset and never touches the field pose, so field-oriented drive and field-pose consumers
 *       stay consistent.</li>
 *   <li><b>Mid-auto pose reset.</b> The heading comes from the pose on every call. A reset to a correct
 *       field pose changes the robot's heading in the field, and the driver frame follows it. The driver's
 *       forward does not move.</li>
 * </ul>
 */
public final class DriverOrientation {
    private static final double PI = Math.PI;

    /** Driver Station alliance as seen by the driver orientation. */
    public enum Alliance {
        BLUE,
        RED
    }

    private final Supplier<PoseSnapshot> fieldPose;
    private final Supplier<Optional<Alliance>> alliance;
    private final BooleanSupplier poseSeeded;
    private final Function<PoseSnapshot, ActionBinding> resetPose;
    private volatile double driverOffsetRadians;

    private DriverOrientation(
            Supplier<PoseSnapshot> fieldPose,
            Supplier<Optional<Alliance>> alliance,
            BooleanSupplier poseSeeded,
            Function<PoseSnapshot, ActionBinding> resetPose) {
        this.fieldPose = Objects.requireNonNull(fieldPose, "fieldPose");
        this.alliance = Objects.requireNonNull(alliance, "alliance");
        this.poseSeeded = Objects.requireNonNull(poseSeeded, "poseSeeded");
        this.resetPose = Objects.requireNonNull(resetPose, "resetPose");
    }

    /**
     * Creates the driver orientation for a robot.
     *
     * @param fieldPose localized field pose, for example {@code localization::pose}
     * @param alliance Driver Station alliance, empty while unknown, for example
     *     {@code () -> DriverStation.getAlliance().map(a -> a == Alliance.Red ? Alliance.RED : Alliance.BLUE)}
     * @param poseSeeded true once auto or vision has set the field pose, for example
     *     {@code () -> autoRan || visionSeeded}
     * @param resetPose resets the localized pose, for example {@code localization::reset}
     * @return driver orientation
     */
    public static DriverOrientation of(
            Supplier<PoseSnapshot> fieldPose,
            Supplier<Optional<Alliance>> alliance,
            BooleanSupplier poseSeeded,
            Function<PoseSnapshot, ActionBinding> resetPose) {
        return new DriverOrientation(fieldPose, alliance, poseSeeded, resetPose);
    }

    /** Returns the field-heading angle the driver calls forward, in {@code (-pi, pi]}. */
    public double forwardRadians() {
        return normalize(allianceForwardRadians() + driverOffsetRadians);
    }

    /**
     * Returns the robot heading relative to driver forward, in {@code (-pi, pi]}. Use it as the heading supplier
     * for field-oriented drive.
     */
    public DoubleSupplier headingRadians() {
        return this::driverHeadingRadians;
    }

    /** Returns the current robot heading relative to driver forward, in {@code (-pi, pi]}. */
    public double driverHeadingRadians() {
        return normalize(fieldHeadingRadians() - forwardRadians());
    }

    /** Returns true once the Driver Station reports an alliance. */
    public boolean allianceKnown() {
        return alliance.get().isPresent();
    }

    /**
     * Driver reset: the robot's current facing becomes forward. Stores an offset only; the field pose is not
     * changed.
     */
    public Action resetHeading() {
        return Actions.run(this::captureForwardFromCurrentHeading);
    }

    /** Clears the driver's reset offset so forward is the alliance default again. */
    public Action restoreAllianceForward() {
        return Actions.run(this::clearDriverOffset);
    }

    /**
     * Sets the pose heading to the alliance forward when the pose has not been seeded, keeping the current
     * translation. Does nothing when the pose is seeded by auto or vision.
     */
    public Action assumeDriverForward() {
        return Actions.run(this::assumeDriverForwardBinding);
    }

    /** Package-private for tests and for {@link #assumeDriverForward()}. */
    ActionBinding assumeDriverForwardBinding() {
        return context -> {
            if (poseSeeded.getAsBoolean()) {
                return;
            }
            PoseSnapshot current = fieldPose.get();
            resetPose.apply(new PoseSnapshot(current.xMeters(), current.yMeters(), allianceForwardRadians()))
                    .apply(context);
        };
    }

    /** Package-private for tests and for {@link #resetHeading()}. */
    void captureForwardFromCurrentHeading() {
        driverOffsetRadians = normalize(fieldHeadingRadians() - allianceForwardRadians());
    }

    /** Package-private for tests and for {@link #restoreAllianceForward()}. */
    void clearDriverOffset() {
        driverOffsetRadians = 0.0;
    }

    private double allianceForwardRadians() {
        return alliance.get().filter(value -> value == Alliance.RED).isPresent() ? PI : 0.0;
    }

    private double fieldHeadingRadians() {
        return fieldPose.get().headingRadians();
    }

    /** Wraps a heading into {@code (-pi, pi]}. */
    static double normalize(double radians) {
        double wrapped = Math.atan2(Math.sin(radians), Math.cos(radians));
        return wrapped <= -PI ? wrapped + 2.0 * PI : wrapped;
    }
}
