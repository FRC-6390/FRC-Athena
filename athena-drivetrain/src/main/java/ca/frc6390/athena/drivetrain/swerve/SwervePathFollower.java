package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.control.PidController;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

/** Kinematics-owned swerve path following with backend-independent samples. */
public final class SwervePathFollower implements TelemetrySource {
    private final SwerveKinematics kinematics;
    private final FollowerBackend backend;
    private final Supplier<Pose2d> pose;
    private final Function<Pose2d, Action> resetPose;
    private final PidGains translationGains;
    private final PidGains headingGains;
    private final PidController xController;
    private final PidController yController;
    private final PidController headingController;
    private final Map<String, TelemetryValue> telemetry;
    private RobotVelocityPool.Channel pooledOutput;
    private Action pooledDrive;

    SwervePathFollower(
            SwerveKinematics kinematics,
            FollowerBackend backend,
            Supplier<Pose2d> pose,
            Function<Pose2d, Action> resetPose,
            PidGains translationGains,
            PidGains headingGains) {
        this.kinematics = Objects.requireNonNull(kinematics, "kinematics");
        this.backend = Objects.requireNonNull(backend, "backend");
        this.pose = Objects.requireNonNull(pose, "pose");
        this.resetPose = Objects.requireNonNull(resetPose, "resetPose");
        this.translationGains = Objects.requireNonNull(translationGains, "translationGains");
        this.headingGains = Objects.requireNonNull(headingGains, "headingGains");
        xController = translationGains.controller();
        yController = translationGains.controller();
        headingController = headingGains.controller().continuous(-Math.PI, Math.PI);
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        translationGains.telemetry().forEach((name, value) -> values.put("translation/" + name, value));
        headingGains.telemetry().forEach((name, value) -> values.put("heading/" + name, value));
        telemetry = Map.copyOf(values);
    }

    public FollowerBackend backend() {
        return backend;
    }

    public PidGains translationGains() {
        return translationGains;
    }

    public PidGains headingGains() {
        return headingGains;
    }

    public Pose2d pose() {
        return Objects.requireNonNull(pose.get(), "pose supplier returned null");
    }

    public Action resetPose(Pose2d target) {
        return Objects.requireNonNull(resetPose.apply(Objects.requireNonNull(target, "target")),
                "resetPose returned null");
    }

    /** Clears controller history when a path or split starts. */
    public void reset() {
        xController.reset();
        yController.reset();
        headingController.reset();
    }

    /** Routes path velocity through a pool channel and its shared drivetrain action. */
    public SwervePathFollower pooled(RobotVelocityPool.Channel output, Action driveAction) {
        pooledOutput = Objects.requireNonNull(output, "output");
        pooledDrive = Objects.requireNonNull(driveAction, "driveAction");
        return this;
    }

    /** Stable drivetrain ownership used to compile path actions without reserving unrelated controls. */
    public List<?> ownership() {
        if (pooledDrive != null) return List.of(pooledDrive);
        return kinematics.modules().stream().map(SwerveKinematics.Module::module).toList();
    }

    /** Clears a configured pooled path contribution. */
    public void stop() {
        if (pooledOutput != null) {
            pooledOutput.clear();
        }
    }

    /** Converts one backend-independent field sample into the normal composed module Action. */
    public Action follow(SwervePathSample sample, double dtSeconds) {
        RobotVelocity velocity = calculateVelocity(sample, dtSeconds);
        if (pooledOutput == null) {
            return kinematics.drive(velocity);
        }
        pooledOutput.set(velocity);
        return pooledDrive;
    }

    RobotVelocity calculateVelocity(SwervePathSample sample, double dtSeconds) {
        Objects.requireNonNull(sample, "sample");
        Pose2d current = pose();
        Pose2d target = sample.pose();
        RobotVelocity feedforward = sample.fieldVelocity();
        double x = feedforward.xMetersPerSecond()
                + xController.calculate(current.getX(), target.getX(), dtSeconds);
        double y = feedforward.yMetersPerSecond()
                + yController.calculate(current.getY(), target.getY(), dtSeconds);
        double omega = feedforward.angularRadiansPerSecond()
                + headingController.calculate(
                        current.getRotation().getRadians(),
                        target.getRotation().getRadians(),
                        dtSeconds);
        return RobotVelocity.field(x, y, omega).fieldToRobot(current.getRotation().getRadians());
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        return telemetry;
    }

}
