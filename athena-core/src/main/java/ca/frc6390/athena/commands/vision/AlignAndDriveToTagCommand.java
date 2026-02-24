package ca.frc6390.athena.commands.vision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera.CoordinateSpace;
import ca.frc6390.athena.sensors.camera.VisionCamera.TargetObservation;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives the robot to a desired range and heading relative to the active localization camera target.
 * The command queries the {@link RobotCore} vision subsystem so it can aggregate readings from any
 * number of configured cameras.
 */
public class AlignAndDriveToTagCommand extends Command {

    private final RobotCore<? extends RobotDrivetrain<?>> robotBase;
    private final RobotSpeeds robotSpeeds;
    private final RobotVision robotVision;
    private final RobotLocalization<?> localization;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;
    private final Pose2d targetOffset;
    private final Pose2d tolerancePose;
    private final double xToleranceMeters;
    private final double yToleranceMeters;
    private final double headingToleranceDegrees;
    private final double maxLinearVelocityMetersPerSec;
    private final double maxAngularVelocityRadPerSec;
    private final double timeoutSeconds;
    private final List<CameraSource> cameraSources;
    private final Set<String> cameraTables;
    private final EnumSet<CameraRole> requiredRoles;

    private double startTimeSeconds;
    private boolean haveMeasurement;

    /**
     * Convenience constructor targeting a single camera table with default motion constraints.
     */
    public AlignAndDriveToTagCommand(
            RobotCore<? extends RobotDrivetrain<?>> robotBase,
            PIDController xController,
            PIDController yController,
            PIDController headingController,
            Pose2d targetOffset,
            Pose2d tolerancePose,
            String cameraTable) {
        this(
                robotBase,
                xController,
                yController,
                headingController,
                targetOffset,
                tolerancePose,
                Double.NaN,
                Double.NaN,
                0.0,
                List.of(Objects.requireNonNull(cameraTable, "cameraTable")),
                EnumSet.noneOf(CameraRole.class));
    }

    /**
     * Convenience constructor targeting a single camera table with explicit motion constraints.
     */
    public AlignAndDriveToTagCommand(
            RobotCore<? extends RobotDrivetrain<?>> robotBase,
            PIDController xController,
            PIDController yController,
            PIDController headingController,
            Pose2d targetOffset,
            Pose2d tolerancePose,
            double maxLinearVelocityMetersPerSec,
            double maxAngularVelocityRadPerSec,
            double timeoutSeconds,
            String cameraTable) {
        this(
                robotBase,
                xController,
                yController,
                headingController,
                targetOffset,
                tolerancePose,
                maxLinearVelocityMetersPerSec,
                maxAngularVelocityRadPerSec,
                timeoutSeconds,
                List.of(Objects.requireNonNull(cameraTable, "cameraTable")),
                EnumSet.noneOf(CameraRole.class));
    }

    /**
     * Convenience constructor targeting a collection of camera tables with default motion constraints.
     */
    public AlignAndDriveToTagCommand(
            RobotCore<? extends RobotDrivetrain<?>> robotBase,
            PIDController xController,
            PIDController yController,
            PIDController headingController,
            Pose2d targetOffset,
            Pose2d tolerancePose,
            Collection<String> cameraTables) {
        this(
                robotBase,
                xController,
                yController,
                headingController,
                targetOffset,
                tolerancePose,
                Double.NaN,
                Double.NaN,
                0.0,
                cameraTables,
                EnumSet.noneOf(CameraRole.class));
    }

    /**
     * Convenience constructor that allows filtering by camera role while using default motion constraints.
     */
    public AlignAndDriveToTagCommand(
            RobotCore<? extends RobotDrivetrain<?>> robotBase,
            PIDController xController,
            PIDController yController,
            PIDController headingController,
            Pose2d targetOffset,
            Pose2d tolerancePose,
            Collection<String> cameraTables,
            EnumSet<CameraRole> cameraRoles) {
        this(
                robotBase,
                xController,
                yController,
                headingController,
                targetOffset,
                tolerancePose,
                Double.NaN,
                Double.NaN,
                0.0,
                cameraTables,
                cameraRoles);
    }

    /**
     * Convenience constructor that targets multiple camera tables without filtering by role.
     */
    public AlignAndDriveToTagCommand(
            RobotCore<? extends RobotDrivetrain<?>> robotBase,
            PIDController xController,
            PIDController yController,
            PIDController headingController,
            Pose2d targetOffset,
            Pose2d tolerancePose,
            double maxLinearVelocityMetersPerSec,
            double maxAngularVelocityRadPerSec,
            double timeoutSeconds,
            Collection<String> cameraTables) {
        this(
                robotBase,
                xController,
                yController,
                headingController,
                targetOffset,
                tolerancePose,
                maxLinearVelocityMetersPerSec,
                maxAngularVelocityRadPerSec,
                timeoutSeconds,
                cameraTables,
                EnumSet.noneOf(CameraRole.class));
    }

    /**
     * Creates an align-and-drive command bound to one or more camera tables.
     *
     * @param robotBase owning robot instance that exposes drivetrain, vision, and localization
     * @param xController PID controller used to regulate the forward distance (robot X axis)
     * @param yController PID controller used to regulate the strafe distance (robot Y axis)
     * @param headingController PID controller configured for heading corrections (degrees domain)
     * @param targetOffset desired robot-to-tag pose expressed in the robot frame
     * @param tolerancePose per-axis tolerances (X/Y in meters, rotation in degrees)
     * @param maxLinearVelocityMetersPerSec clamp for commanded chassis X/Y velocity (values {@code <= 0} disable clamping)
     * @param maxAngularVelocityRadPerSec clamp for commanded chassis angular velocity (values {@code <= 0} disable clamping)
     * @param timeoutSeconds command timeout in seconds ({@code <= 0} disables the timeout)
     * @param cameraTables required collection of camera table names to consider
     * @param cameraRoles optional set of roles; cameras must match at least one role when provided
     */
    public AlignAndDriveToTagCommand(
            RobotCore<? extends RobotDrivetrain<?>> robotBase,
            PIDController xController,
            PIDController yController,
            PIDController headingController,
            Pose2d targetOffset,
            Pose2d tolerancePose,
            double maxLinearVelocityMetersPerSec,
            double maxAngularVelocityRadPerSec,
            double timeoutSeconds,
            Collection<String> cameraTables,
            EnumSet<CameraRole> cameraRoles) {
        this.robotBase = Objects.requireNonNull(robotBase, "robotBase");
        this.xController = Objects.requireNonNull(xController, "xController");
        this.yController = Objects.requireNonNull(yController, "yController");
        this.headingController = Objects.requireNonNull(headingController, "headingController");
        this.targetOffset = Objects.requireNonNull(targetOffset, "targetOffset");
        this.tolerancePose = Objects.requireNonNull(tolerancePose, "tolerancePose");
        this.xToleranceMeters = AlignAndDriveToTagPolicy.sanitizeToleranceMeters(tolerancePose.getX());
        this.yToleranceMeters = AlignAndDriveToTagPolicy.sanitizeToleranceMeters(tolerancePose.getY());
        this.headingToleranceDegrees = AlignAndDriveToTagPolicy.sanitizeToleranceDegrees(tolerancePose.getRotation().getDegrees());
        this.maxLinearVelocityMetersPerSec =
                maxLinearVelocityMetersPerSec > 0.0 ? maxLinearVelocityMetersPerSec : Double.POSITIVE_INFINITY;
        this.maxAngularVelocityRadPerSec =
                maxAngularVelocityRadPerSec > 0.0 ? maxAngularVelocityRadPerSec : Double.POSITIVE_INFINITY;
        this.timeoutSeconds = timeoutSeconds;

        RobotDrivetrain<?> drivetrain = Objects.requireNonNull(robotBase.drivetrain(), "robotBase.drivetrain()");
        this.robotSpeeds = Objects.requireNonNull(drivetrain.robotSpeeds(), "robotSpeeds");
        this.robotVision = robotBase.vision();
        if (robotVision == null) {
            throw new IllegalArgumentException("Robot base does not expose a vision system.");
        }
        this.localization = robotBase.localization();

        this.cameraSources = new ArrayList<>();
        this.cameraTables = AlignAndDriveToTagPolicy.sanitizeCameraTables(cameraTables);
        this.requiredRoles = AlignAndDriveToTagPolicy.sanitizeRoles(cameraRoles);

        headingController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        headingController.reset();

        xController.setSetpoint(targetOffset.getX());
        yController.setSetpoint(targetOffset.getY());
        headingController.setSetpoint(targetOffset.getRotation().getDegrees());

        xController.setTolerance(xToleranceMeters);
        yController.setTolerance(yToleranceMeters);
        headingController.setTolerance(headingToleranceDegrees);

        startTimeSeconds = Timer.getFPGATimestamp();
        haveMeasurement = false;
        updateCameraSources();
    }

    @Override
    public void execute() {
        updateCameraSources();
        Pose2d robotPose = localization != null ? localization.getFieldPose() : null;
        SelectedObservation selection = selectBestObservation(robotPose);

        if (selection == null) {
            haveMeasurement = false;
            robotSpeeds.stopSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
            return;
        }

        TargetObservation observation = selection.observation();
        Translation2d translation = observation.translation();
        if (translation == null || !observation.hasYaw()) {
            haveMeasurement = false;
            robotSpeeds.stopSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
            return;
        }

        double yawDegrees = observation.yawDegrees();
        haveMeasurement = true;

        double vxMetersPerSec = -xController.calculate(translation.getX());
        double vyMetersPerSec = -yController.calculate(translation.getY());
        double omegaRadPerSec = -Math.toRadians(headingController.calculate(yawDegrees));

        double clampedVx = MathUtil.clamp(vxMetersPerSec, -maxLinearVelocityMetersPerSec, maxLinearVelocityMetersPerSec);
        double clampedVy = MathUtil.clamp(vyMetersPerSec, -maxLinearVelocityMetersPerSec, maxLinearVelocityMetersPerSec);
        double clampedOmega =
                MathUtil.clamp(omegaRadPerSec, -maxAngularVelocityRadPerSec, maxAngularVelocityRadPerSec);

        robotSpeeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, clampedVx, clampedVy, clampedOmega);
    }

    @Override
    public void end(boolean interrupted) {
        robotSpeeds.stopSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
    }

    @Override
    public boolean isFinished() {
        if (timeoutSeconds > 0.0) {
            double elapsed = Timer.getFPGATimestamp() - startTimeSeconds;
            if (elapsed >= timeoutSeconds) {
                return true;
            }
        }

        if (!haveMeasurement) {
            return false;
        }

        boolean translationOnTarget = xController.atSetpoint() && yController.atSetpoint();
        boolean headingOnTarget = headingController.atSetpoint();
        return translationOnTarget && headingOnTarget;
    }

    private void updateCameraSources() {
        cameraSources.clear();
        Map<String, VisionCamera> cameras = robotVision.cameras().all();
        if (cameras.isEmpty()) {
            return;
        }

        for (String key : cameraTables) {
            VisionCamera camera = cameras.get(key);
            if (camera != null && cameraMatches(camera)) {
                cameraSources.add(new CameraSource(key, camera));
            }
        }
    }

    private boolean cameraMatches(VisionCamera camera) {
        if (requiredRoles.isEmpty()) {
            return true;
        }
        EnumSet<CameraRole> roles = camera.getRoles();
        for (CameraRole role : requiredRoles) {
            if (roles.contains(role)) {
                return true;
            }
        }
        return false;
    }

    private SelectedObservation selectBestObservation(Pose2d robotPose) {
        TargetObservation best = null;
        String bestKey = null;

        for (CameraSource source : cameraSources) {
            Optional<TargetObservation> observationOpt =
                    source.camera().getLatestObservation(CoordinateSpace.ROBOT, robotPose);
            if (observationOpt.isEmpty()) {
                continue;
            }
            TargetObservation observation = observationOpt.get();
            if (!observation.hasTranslation() || !observation.hasYaw()) {
                continue;
            }
            TargetObservation candidate = AlignAndDriveToTagPolicy.pickBetterObservation(best, observation);
            if (candidate == observation) {
                best = observation;
                bestKey = source.key();
            }
        }

        if (best != null) {
            return new SelectedObservation(bestKey, best);
        }

        return null;
    }

    private record CameraSource(String key, VisionCamera camera) {}

    private record SelectedObservation(String cameraKey, TargetObservation observation) {}
}
