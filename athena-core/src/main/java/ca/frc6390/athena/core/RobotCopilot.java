package ca.frc6390.athena.core;

import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.core.auto.AutoBackends;
import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCopilot {

    public enum DriveStyle {
        HOLONOMIC,
        DIFFERENTIAL
    }

    public record DriveToPoseConfig(
            HolonomicPidConstants translationPid,
            HolonomicPidConstants rotationPid,
            Pose2d tolerance,
            Supplier<MotionLimits.DriveLimits> constraintsSupplier,
            String speedSource,
            double timeoutSeconds) {
        public DriveToPoseConfig {
            translationPid = translationPid != null ? translationPid : new HolonomicPidConstants(0, 0, 0);
            rotationPid = rotationPid != null ? rotationPid : new HolonomicPidConstants(0, 0, 0);
            tolerance = tolerance != null ? tolerance : DEFAULT_TOLERANCE;
            constraintsSupplier = constraintsSupplier != null ? constraintsSupplier : MotionLimits.DriveLimits::none;
            speedSource = (speedSource != null && !speedSource.isBlank()) ? speedSource : DEFAULT_SPEED_SOURCE;
            timeoutSeconds = Double.isFinite(timeoutSeconds) ? timeoutSeconds : 0.0;
        }
    }

    private static final Pose2d DEFAULT_TOLERANCE =
            new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2.0));
    private static final String DEFAULT_SPEED_SOURCE = "feedback";

    private final RobotDrivetrain<?> drivetrain;
    private final RobotLocalization<?> localization;
    private final RobotSpeeds robotSpeeds;
    private final DriveStyle driveStyle;
    private final HolonomicPidConstants defaultTranslationPid;
    private final HolonomicPidConstants defaultRotationPid;
    private final MotionLimits motionLimits;

    public RobotCopilot(RobotDrivetrain<?> drivetrain, RobotLocalization<?> localization, DriveStyle driveStyle) {
        this(drivetrain, localization, driveStyle, null);
    }

    public RobotCopilot(RobotDrivetrain<?> drivetrain,
                        RobotLocalization<?> localization,
                        DriveStyle driveStyle,
                        MotionLimits motionLimits) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain");
        this.localization = localization;
        this.robotSpeeds = drivetrain.robotSpeeds();
        this.driveStyle = driveStyle != null ? driveStyle : DriveStyle.HOLONOMIC;
        RobotLocalizationConfig config = localization != null ? localization.getLocalizationConfig() : null;
        this.defaultTranslationPid = config != null ? config.translation() : new HolonomicPidConstants(0, 0, 0);
        this.defaultRotationPid = config != null ? config.rotation() : new HolonomicPidConstants(0, 0, 0);
        this.motionLimits = motionLimits != null ? motionLimits : resolveMotionLimits(drivetrain);
    }

    public static RobotCopilot from(RobotCore<?> core) {
        Objects.requireNonNull(core, "core");
        RobotDrivetrain<?> drivetrain = core.drivetrain();
        RobotLocalization<?> localization = core.localization();
        return new RobotCopilot(drivetrain, localization, inferDriveStyle(drivetrain));
    }

    public static DriveStyle inferDriveStyle(RobotDrivetrain<?> drivetrain) {
        if (drivetrain instanceof SwerveDrivetrain) {
            return DriveStyle.HOLONOMIC;
        }
        if (drivetrain instanceof DifferentialDrivetrain) {
            return DriveStyle.DIFFERENTIAL;
        }
        return DriveStyle.HOLONOMIC;
    }

    public DriveStyle getDriveStyle() {
        return driveStyle;
    }

    private static MotionLimits resolveMotionLimits(RobotDrivetrain<?> drivetrain) {
        MotionLimits limits = drivetrain.speeds().limits();
        return limits != null ? limits : new MotionLimits();
    }

    public DriveToPoseConfig defaultDriveToPoseConfig() {
        return new DriveToPoseConfig(
                defaultTranslationPid,
                defaultRotationPid,
                DEFAULT_TOLERANCE,
                this::resolveDriveLimits,
                DEFAULT_SPEED_SOURCE,
                0.0);
    }

    public Command driveToPose(Pose2d target) {
        return driveToPose(() -> target, defaultDriveToPoseConfig());
    }

    public Command driveToPose(Supplier<Pose2d> targetSupplier, DriveToPoseConfig config) {
        Objects.requireNonNull(targetSupplier, "targetSupplier");
        DriveToPoseConfig resolved = config != null ? config : defaultDriveToPoseConfig();
        if (localization == null) {
            return Commands.runOnce(() ->
                    DriverStation.reportWarning("RobotCopilot driveToPose requires localization.", false));
        }
        return new DriveToPoseCommand(targetSupplier, resolved);
    }

    public Command followPathPlannerAuto(String autoName) {
        if (autoName == null || autoName.isBlank()) {
            return Commands.none();
        }
        return AutoBackends.forSource(RobotAuto.AutoSource.PATH_PLANNER)
                .flatMap(backend -> backend.buildAuto(RobotAuto.AutoSource.PATH_PLANNER, autoName))
                .orElseGet(() -> {
                    DriverStation.reportWarning(
                            "Path planner backend missing; auto \"" + autoName + "\" unavailable.", false);
                    return Commands.none();
                });
    }

    public Command followChoreo(String trajectoryName) {
        if (trajectoryName == null || trajectoryName.isBlank()) {
            return Commands.none();
        }
        return AutoBackends.forSource(RobotAuto.AutoSource.CHOREO)
                .flatMap(backend -> backend.buildAuto(RobotAuto.AutoSource.CHOREO, trajectoryName))
                .orElseGet(() -> {
                    DriverStation.reportWarning(
                            "Choreo backend missing; auto \"" + trajectoryName + "\" unavailable.", false);
                    return Commands.none();
                });
    }

    public void stop() {
        robotSpeeds.stopSpeeds(DEFAULT_SPEED_SOURCE);
    }

    public void stop(String speedSource) {
        robotSpeeds.stopSpeeds(speedSource != null ? speedSource : DEFAULT_SPEED_SOURCE);
    }

    private final class DriveToPoseCommand extends Command {
        private final Supplier<Pose2d> targetSupplier;
        private final DriveToPoseConfig config;
        private final PIDController xController;
        private final PIDController yController;
        private final PIDController distanceController;
        private final PIDController headingController;
        private boolean abort;
        private double startTimeSeconds;
        private double lastTimestampSeconds = Double.NaN;
        private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

        private DriveToPoseCommand(Supplier<Pose2d> targetSupplier, DriveToPoseConfig config) {
            this.targetSupplier = targetSupplier;
            this.config = config;
            HolonomicPidConstants translationPid = config.translationPid();
            HolonomicPidConstants rotationPid = config.rotationPid();
            xController = new PIDController(translationPid.kP(), translationPid.kI(), translationPid.kD());
            yController = new PIDController(translationPid.kP(), translationPid.kI(), translationPid.kD());
            distanceController = new PIDController(translationPid.kP(), translationPid.kI(), translationPid.kD());
            headingController = new PIDController(rotationPid.kP(), rotationPid.kI(), rotationPid.kD());
            headingController.enableContinuousInput(-Math.PI, Math.PI);
            addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
            abort = false;
            startTimeSeconds = Timer.getFPGATimestamp();
            lastTimestampSeconds = Double.NaN;
            lastSpeeds = new ChassisSpeeds();
            xController.reset();
            yController.reset();
            distanceController.reset();
            headingController.reset();
            Pose2d tolerance = config.tolerance();
            xController.setTolerance(Math.abs(tolerance.getX()));
            yController.setTolerance(Math.abs(tolerance.getY()));
            distanceController.setTolerance(Math.max(Math.abs(tolerance.getX()), Math.abs(tolerance.getY())));
            headingController.setTolerance(Math.abs(tolerance.getRotation().getRadians()));
        }

        @Override
        public void execute() {
            Pose2d target = targetSupplier.get();
            if (target == null) {
                abort = true;
                robotSpeeds.stopSpeeds(config.speedSource());
                return;
            }
            Pose2d current = localization.getFieldPose();
            ChassisSpeeds desiredSpeeds;
            if (driveStyle == DriveStyle.DIFFERENTIAL) {
                Translation2d toTarget = target.getTranslation().minus(current.getTranslation());
                double distance = toTarget.getNorm();
                Rotation2d desiredHeading = target.getRotation();
                double linear = distanceController.calculate(0.0, distance);
                double omega = headingController.calculate(
                        current.getRotation().getRadians(),
                        desiredHeading.getRadians());
                desiredSpeeds = new ChassisSpeeds(linear, 0.0, omega);
            } else {
                double vx = xController.calculate(current.getX(), target.getX());
                double vy = yController.calculate(current.getY(), target.getY());
                double omega = headingController.calculate(
                        current.getRotation().getRadians(),
                        target.getRotation().getRadians());
                ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, omega);
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldRelative,
                        current.getRotation());
            }
            MotionLimits.DriveLimits constraints = resolveConstraints();
            double now = Timer.getFPGATimestamp();
            ChassisSpeeds limited = applyConstraints(desiredSpeeds, constraints, now);
            robotSpeeds.setSpeeds(config.speedSource(), limited);
        }

        @Override
        public void end(boolean interrupted) {
            robotSpeeds.stopSpeeds(config.speedSource());
        }

        @Override
        public boolean isFinished() {
            if (abort) {
                return true;
            }
            if (config.timeoutSeconds() > 0.0
                    && Timer.getFPGATimestamp() - startTimeSeconds >= config.timeoutSeconds()) {
                return true;
            }
            Pose2d target = targetSupplier.get();
            if (target == null) {
                return true;
            }
            return withinTolerance(localization.getFieldPose(), target, config.tolerance());
        }

        private MotionLimits.DriveLimits resolveConstraints() {
            Supplier<MotionLimits.DriveLimits> supplier = config.constraintsSupplier();
            if (supplier == null) {
                return MotionLimits.DriveLimits.none();
            }
            MotionLimits.DriveLimits constraints = supplier.get();
            return constraints != null ? constraints : MotionLimits.DriveLimits.none();
        }

        private boolean withinTolerance(Pose2d current, Pose2d target, Pose2d tolerance) {
            double dx = target.getX() - current.getX();
            double dy = target.getY() - current.getY();
            double dTheta = Math.abs(target.getRotation().minus(current.getRotation()).getRadians());
            return Math.abs(dx) <= Math.abs(tolerance.getX())
                    && Math.abs(dy) <= Math.abs(tolerance.getY())
                    && dTheta <= Math.abs(tolerance.getRotation().getRadians());
        }

        private ChassisSpeeds applyConstraints(ChassisSpeeds speeds,
                                               MotionLimits.DriveLimits constraints,
                                               double nowSeconds) {
            ChassisSpeeds limited = limitVelocity(speeds, constraints);
            limited = limitAcceleration(limited, constraints, nowSeconds);
            lastSpeeds = limited;
            return limited;
        }

        private ChassisSpeeds limitVelocity(ChassisSpeeds speeds, MotionLimits.DriveLimits constraints) {
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;
            double omega = speeds.omegaRadiansPerSecond;
            if (constraints.maxLinearVelocity() > 0.0) {
                double linear = Math.hypot(vx, vy);
                if (linear > constraints.maxLinearVelocity()) {
                    double scale = constraints.maxLinearVelocity() / linear;
                    vx *= scale;
                    vy *= scale;
                }
            }
            if (constraints.maxAngularVelocity() > 0.0) {
                omega = MathUtil.clamp(omega, -constraints.maxAngularVelocity(), constraints.maxAngularVelocity());
            }
            return new ChassisSpeeds(vx, vy, omega);
        }

        private ChassisSpeeds limitAcceleration(ChassisSpeeds speeds,
                                                MotionLimits.DriveLimits constraints,
                                                double nowSeconds) {
            if (!Double.isFinite(lastTimestampSeconds)) {
                lastTimestampSeconds = nowSeconds;
                return speeds;
            }
            double dt = nowSeconds - lastTimestampSeconds;
            lastTimestampSeconds = nowSeconds;
            if (dt <= 0.0) {
                return speeds;
            }
            double maxLinearDelta = constraints.maxLinearAcceleration() > 0.0
                    ? constraints.maxLinearAcceleration() * dt
                    : Double.POSITIVE_INFINITY;
            double maxAngularDelta = constraints.maxAngularAcceleration() > 0.0
                    ? constraints.maxAngularAcceleration() * dt
                    : Double.POSITIVE_INFINITY;
            double vx = limitDelta(speeds.vxMetersPerSecond, lastSpeeds.vxMetersPerSecond, maxLinearDelta);
            double vy = limitDelta(speeds.vyMetersPerSecond, lastSpeeds.vyMetersPerSecond, maxLinearDelta);
            double omega = limitDelta(speeds.omegaRadiansPerSecond, lastSpeeds.omegaRadiansPerSecond, maxAngularDelta);
            return new ChassisSpeeds(vx, vy, omega);
        }

        private double limitDelta(double value, double lastValue, double maxDelta) {
            if (!Double.isFinite(maxDelta)) {
                return value;
            }
            double delta = value - lastValue;
            if (delta > maxDelta) {
                return lastValue + maxDelta;
            }
            if (delta < -maxDelta) {
                return lastValue - maxDelta;
            }
            return value;
        }
    }

    private MotionLimits.DriveLimits resolveDriveLimits() {
        if (motionLimits != null) {
            return motionLimits.resolveDriveLimits();
        }
        return MotionLimits.DriveLimits.fromRobotSpeeds(robotSpeeds);
    }
}
