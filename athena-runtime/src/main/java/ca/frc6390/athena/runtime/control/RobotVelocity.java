package ca.frc6390.athena.runtime.control;

import java.util.Objects;

/**
 * Immutable planar chassis velocity with an explicit coordinate frame.
 *
 * @param xMetersPerSecond forward or down-field velocity
 * @param yMetersPerSecond left or across-field velocity
 * @param angularRadiansPerSecond counter-clockwise angular velocity
 * @param frame coordinate frame of the linear components
 */
public record RobotVelocity(
        double xMetersPerSecond,
        double yMetersPerSecond,
        double angularRadiansPerSecond,
        VelocityFrame frame) {
    public RobotVelocity {
        xMetersPerSecond = finiteOrZero(xMetersPerSecond);
        yMetersPerSecond = finiteOrZero(yMetersPerSecond);
        angularRadiansPerSecond = finiteOrZero(angularRadiansPerSecond);
        frame = Objects.requireNonNull(frame, "frame");
    }

    /** Creates a robot-relative velocity for source compatibility. */
    public RobotVelocity(double xMetersPerSecond, double yMetersPerSecond, double angularRadiansPerSecond) {
        this(xMetersPerSecond, yMetersPerSecond, angularRadiansPerSecond, VelocityFrame.ROBOT);
    }

    public static RobotVelocity robot(double x, double y, double angular) {
        return new RobotVelocity(x, y, angular, VelocityFrame.ROBOT);
    }

    public static RobotVelocity field(double x, double y, double angular) {
        return new RobotVelocity(x, y, angular, VelocityFrame.FIELD);
    }

    public static RobotVelocity zero() {
        return zero(VelocityFrame.ROBOT);
    }

    public static RobotVelocity zero(VelocityFrame frame) {
        return new RobotVelocity(0.0, 0.0, 0.0, frame);
    }

    public static RobotVelocity linear(double x, double y) {
        return robot(x, y, 0.0);
    }

    public static RobotVelocity linear(double x, double y, VelocityFrame frame) {
        return new RobotVelocity(x, y, 0.0, frame);
    }

    public static RobotVelocity angular(double angular) {
        return robot(0.0, 0.0, angular);
    }

    public static RobotVelocity angular(double angular, VelocityFrame frame) {
        return new RobotVelocity(0.0, 0.0, angular, frame);
    }

    public RobotVelocity plus(RobotVelocity other) {
        RobotVelocity safe = requireSameFrame(other);
        return new RobotVelocity(
                xMetersPerSecond + safe.xMetersPerSecond,
                yMetersPerSecond + safe.yMetersPerSecond,
                angularRadiansPerSecond + safe.angularRadiansPerSecond,
                frame);
    }

    public RobotVelocity minus(RobotVelocity other) {
        return plus(Objects.requireNonNull(other, "other").unaryMinus());
    }

    public RobotVelocity times(double scalar) {
        double safe = finiteOrZero(scalar);
        return new RobotVelocity(
                xMetersPerSecond * safe,
                yMetersPerSecond * safe,
                angularRadiansPerSecond * safe,
                frame);
    }

    public RobotVelocity div(double scalar) {
        if (!Double.isFinite(scalar) || scalar == 0.0) {
            throw new IllegalArgumentException("Velocity divisor must be finite and non-zero.");
        }
        return times(1.0 / scalar);
    }

    public RobotVelocity unaryMinus() {
        return times(-1.0);
    }

    public RobotVelocity withLinear(double x, double y) {
        return new RobotVelocity(x, y, angularRadiansPerSecond, frame);
    }

    public RobotVelocity withAngular(double angular) {
        return new RobotVelocity(xMetersPerSecond, yMetersPerSecond, angular, frame);
    }

    public RobotVelocity linearOnly() {
        return withAngular(0.0);
    }

    public RobotVelocity angularOnly() {
        return new RobotVelocity(0.0, 0.0, angularRadiansPerSecond, frame);
    }

    public double linearMagnitude() {
        return Math.hypot(xMetersPerSecond, yMetersPerSecond);
    }

    public RobotVelocity interpolate(RobotVelocity end, double amount) {
        RobotVelocity safe = requireSameFrame(end);
        double t = Math.max(0.0, Math.min(1.0, finiteOrZero(amount)));
        return plus(safe.minus(this).times(t));
    }

    /** Converts this field-relative value into robot-relative coordinates. */
    public RobotVelocity fieldToRobot(double headingRadians) {
        requireFrame(VelocityFrame.FIELD, "fieldToRobot");
        double heading = finiteOrZero(headingRadians);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return robot(
                xMetersPerSecond * cos + yMetersPerSecond * sin,
                -xMetersPerSecond * sin + yMetersPerSecond * cos,
                angularRadiansPerSecond);
    }

    /** Converts this robot-relative value into field-relative coordinates. */
    public RobotVelocity robotToField(double headingRadians) {
        requireFrame(VelocityFrame.ROBOT, "robotToField");
        double heading = finiteOrZero(headingRadians);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return field(
                xMetersPerSecond * cos - yMetersPerSecond * sin,
                xMetersPerSecond * sin + yMetersPerSecond * cos,
                angularRadiansPerSecond);
    }

    public RobotVelocity inFrame(VelocityFrame target, double headingRadians) {
        VelocityFrame safeTarget = Objects.requireNonNull(target, "target");
        if (frame == safeTarget) {
            return this;
        }
        return safeTarget == VelocityFrame.ROBOT
                ? fieldToRobot(headingRadians)
                : robotToField(headingRadians);
    }

    public RobotVelocity clamp(double maxLinearMetersPerSecond, double maxAngularRadiansPerSecond) {
        double x = xMetersPerSecond;
        double y = yMetersPerSecond;
        double maxLinear = finiteOrZero(maxLinearMetersPerSecond);
        double magnitude = Math.hypot(x, y);
        if (maxLinear > 0.0 && magnitude > maxLinear) {
            double scale = maxLinear / magnitude;
            x *= scale;
            y *= scale;
        }
        double maxAngular = finiteOrZero(maxAngularRadiansPerSecond);
        double angular = angularRadiansPerSecond;
        if (maxAngular > 0.0) {
            angular = Math.max(-maxAngular, Math.min(maxAngular, angular));
        }
        return new RobotVelocity(x, y, angular, frame);
    }

    private RobotVelocity requireSameFrame(RobotVelocity other) {
        RobotVelocity safe = Objects.requireNonNull(other, "other");
        if (frame != safe.frame) {
            throw new IllegalArgumentException(
                    "Cannot combine " + frame + " and " + safe.frame + " robot velocities.");
        }
        return safe;
    }

    private void requireFrame(VelocityFrame required, String operation) {
        if (frame != required) {
            throw new IllegalStateException(operation + " requires a " + required + " velocity, got " + frame + '.');
        }
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
