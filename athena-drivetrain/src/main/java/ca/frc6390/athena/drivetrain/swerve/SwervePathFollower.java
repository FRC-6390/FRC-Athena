package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedHashMap;
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
    private final AxisController xController = new AxisController(false);
    private final AxisController yController = new AxisController(false);
    private final AxisController headingController = new AxisController(true);
    private final Map<String, TelemetryValue> telemetry;

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

    /** Converts one backend-independent field sample into the normal composed module Action. */
    public Action follow(SwervePathSample sample, double dtSeconds) {
        return kinematics.drive(calculateVelocity(sample, dtSeconds));
    }

    RobotVelocity calculateVelocity(SwervePathSample sample, double dtSeconds) {
        Objects.requireNonNull(sample, "sample");
        Pose2d current = pose();
        Pose2d target = sample.pose();
        RobotVelocity feedforward = sample.fieldVelocity();
        double x = feedforward.xMetersPerSecond()
                + xController.calculate(current.getX(), target.getX(), dtSeconds, translationGains);
        double y = feedforward.yMetersPerSecond()
                + yController.calculate(current.getY(), target.getY(), dtSeconds, translationGains);
        double omega = feedforward.angularRadiansPerSecond()
                + headingController.calculate(
                        current.getRotation().getRadians(),
                        target.getRotation().getRadians(),
                        dtSeconds,
                        headingGains);
        return new RobotVelocity(x, y, omega).fieldToRobot(current.getRotation().getRadians());
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        return telemetry;
    }

    private static final class AxisController {
        private final boolean continuous;
        private double integral;
        private double previousError;
        private boolean first = true;

        private AxisController(boolean continuous) {
            this.continuous = continuous;
        }

        private void reset() {
            integral = 0.0;
            previousError = 0.0;
            first = true;
        }

        private double calculate(double measurement, double target, double dt, PidGains gains) {
            if (gains.isDisabled()) return 0.0;
            double error = target - measurement;
            if (continuous) error = MathUtil.inputModulus(error, -Math.PI, Math.PI);
            double safeDt = Double.isFinite(dt) && dt > 0.0 ? dt : 0.0;
            if (safeDt > 0.0 && (gains.iZone() <= 0.0 || Math.abs(error) <= gains.iZone())) {
                integral += error * safeDt;
            } else if (gains.iZone() > 0.0 && Math.abs(error) > gains.iZone()) {
                integral = 0.0;
            }
            double derivative = first || safeDt == 0.0 ? 0.0 : (error - previousError) / safeDt;
            first = false;
            previousError = error;
            return gains.p() * error + gains.i() * integral + gains.d() * derivative;
        }
    }
}
