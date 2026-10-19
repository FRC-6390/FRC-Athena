package ca.frc6390.athena.runtime.control;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.Objects;

/**
 * Immutable planar chassis velocity with an explicit coordinate frame.
 *
 * <p>Team-facing factories and accessors use WPILib units. Components are stored as primitives so
 * that per-cycle math does not allocate measures; the {@code ...Primitive} members exist for runtime
 * hot paths (kinematics, localization, path followers) and are not meant for robot code.
 */
public final class RobotVelocity {
    private final double xMetersPerSecond;
    private final double yMetersPerSecond;
    private final double angularRadiansPerSecond;
    private final VelocityFrame frame;

    private RobotVelocity(
            double xMetersPerSecond,
            double yMetersPerSecond,
            double angularRadiansPerSecond,
            VelocityFrame frame) {
        this.xMetersPerSecond = finiteOrZero(xMetersPerSecond);
        this.yMetersPerSecond = finiteOrZero(yMetersPerSecond);
        this.angularRadiansPerSecond = finiteOrZero(angularRadiansPerSecond);
        this.frame = Objects.requireNonNull(frame, "frame");
    }

    /** Creates a robot-relative velocity from WPILib-typed components. */
    public static RobotVelocity robot(LinearVelocity x, LinearVelocity y, AngularVelocity angular) {
        return new RobotVelocity(mps(x), mps(y), rps(angular), VelocityFrame.ROBOT);
    }

    /** Creates a field-relative velocity from WPILib-typed components. */
    public static RobotVelocity field(LinearVelocity x, LinearVelocity y, AngularVelocity angular) {
        return new RobotVelocity(mps(x), mps(y), rps(angular), VelocityFrame.FIELD);
    }

    /**
     * Creates a velocity from primitive components in meters per second and radians per second.
     * Runtime hot paths only; robot code uses {@link #robot} or {@link #field}.
     */
    public static RobotVelocity ofPrimitives(
            double xMetersPerSecond,
            double yMetersPerSecond,
            double angularRadiansPerSecond,
            VelocityFrame frame) {
        return new RobotVelocity(xMetersPerSecond, yMetersPerSecond, angularRadiansPerSecond, frame);
    }

    public static RobotVelocity zero() {
        return zero(VelocityFrame.ROBOT);
    }

    public static RobotVelocity zero(VelocityFrame frame) {
        return new RobotVelocity(0.0, 0.0, 0.0, frame);
    }

    public static RobotVelocity linear(LinearVelocity x, LinearVelocity y) {
        return robot(x, y, RadiansPerSecond.zero());
    }

    public static RobotVelocity linear(LinearVelocity x, LinearVelocity y, VelocityFrame frame) {
        return new RobotVelocity(mps(x), mps(y), 0.0, frame);
    }

    public static RobotVelocity angular(AngularVelocity angular) {
        return robot(MetersPerSecond.zero(), MetersPerSecond.zero(), angular);
    }

    public static RobotVelocity angular(AngularVelocity angular, VelocityFrame frame) {
        return new RobotVelocity(0.0, 0.0, rps(angular), frame);
    }

    /** Returns the frame of the linear components. */
    public VelocityFrame frame() {
        return frame;
    }

    /** Returns forward (robot) or down-field (field) velocity. */
    public LinearVelocity x() {
        return MetersPerSecond.of(xMetersPerSecond);
    }

    /** Returns left (robot) or across-field (field) velocity. */
    public LinearVelocity y() {
        return MetersPerSecond.of(yMetersPerSecond);
    }

    /** Returns counter-clockwise angular velocity. */
    public AngularVelocity angular() {
        return RadiansPerSecond.of(angularRadiansPerSecond);
    }

    /** Primitive forward component in meters per second. Runtime hot paths only. */
    public double xMetersPerSecondPrimitive() {
        return xMetersPerSecond;
    }

    /** Primitive lateral component in meters per second. Runtime hot paths only. */
    public double yMetersPerSecondPrimitive() {
        return yMetersPerSecond;
    }

    /** Primitive angular component in radians per second. Runtime hot paths only. */
    public double angularRadiansPerSecondPrimitive() {
        return angularRadiansPerSecond;
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

    /** Scales every component by a unitless factor. */
    public RobotVelocity times(double scalar) {
        double safe = finiteOrZero(scalar);
        return new RobotVelocity(
                xMetersPerSecond * safe,
                yMetersPerSecond * safe,
                angularRadiansPerSecond * safe,
                frame);
    }

    /** Divides every component by a unitless divisor. */
    public RobotVelocity div(double scalar) {
        if (!Double.isFinite(scalar) || scalar == 0.0) {
            throw new IllegalArgumentException("Velocity divisor must be finite and non-zero.");
        }
        return times(1.0 / scalar);
    }

    public RobotVelocity unaryMinus() {
        return times(-1.0);
    }

    /** Returns this velocity with every component reversed. */
    public RobotVelocity inverted() {
        return unaryMinus();
    }

    /**
     * Returns this velocity with the selected components reversed.
     *
     * @param x whether to reverse forward velocity
     * @param y whether to reverse lateral velocity
     * @param angular whether to reverse angular velocity
     */
    public RobotVelocity inverted(boolean x, boolean y, boolean angular) {
        return new RobotVelocity(
                x ? -xMetersPerSecond : xMetersPerSecond,
                y ? -yMetersPerSecond : yMetersPerSecond,
                angular ? -angularRadiansPerSecond : angularRadiansPerSecond,
                frame);
    }

    public RobotVelocity withLinear(LinearVelocity x, LinearVelocity y) {
        return new RobotVelocity(mps(x), mps(y), angularRadiansPerSecond, frame);
    }

    public RobotVelocity withAngular(AngularVelocity angular) {
        return new RobotVelocity(xMetersPerSecond, yMetersPerSecond, rps(angular), frame);
    }

    public RobotVelocity linearOnly() {
        return new RobotVelocity(xMetersPerSecond, yMetersPerSecond, 0.0, frame);
    }

    public RobotVelocity angularOnly() {
        return new RobotVelocity(0.0, 0.0, angularRadiansPerSecond, frame);
    }

    /** Returns the magnitude of the linear components. */
    public LinearVelocity linearMagnitude() {
        return MetersPerSecond.of(Math.hypot(xMetersPerSecond, yMetersPerSecond));
    }

    /** Interpolates toward {@code end}; {@code amount} is a unitless fraction clamped to [0, 1]. */
    public RobotVelocity interpolate(RobotVelocity end, double amount) {
        RobotVelocity safe = requireSameFrame(end);
        double t = Math.max(0.0, Math.min(1.0, finiteOrZero(amount)));
        return plus(safe.minus(this).times(t));
    }

    /** Converts this field-relative value into robot-relative coordinates. */
    public RobotVelocity fieldToRobot(Rotation2d heading) {
        requireFrame(VelocityFrame.FIELD, "fieldToRobot");
        double cos = Math.cos(radians(heading));
        double sin = Math.sin(radians(heading));
        return new RobotVelocity(
                xMetersPerSecond * cos + yMetersPerSecond * sin,
                -xMetersPerSecond * sin + yMetersPerSecond * cos,
                angularRadiansPerSecond,
                VelocityFrame.ROBOT);
    }

    /** Converts this robot-relative value into field-relative coordinates. */
    public RobotVelocity robotToField(Rotation2d heading) {
        requireFrame(VelocityFrame.ROBOT, "robotToField");
        double cos = Math.cos(radians(heading));
        double sin = Math.sin(radians(heading));
        return new RobotVelocity(
                xMetersPerSecond * cos - yMetersPerSecond * sin,
                xMetersPerSecond * sin + yMetersPerSecond * cos,
                angularRadiansPerSecond,
                VelocityFrame.FIELD);
    }

    /** Returns this velocity expressed in {@code target}, rotating with {@code heading} if needed. */
    public RobotVelocity inFrame(VelocityFrame target, Rotation2d heading) {
        VelocityFrame safeTarget = Objects.requireNonNull(target, "target");
        if (frame == safeTarget) {
            return this;
        }
        return safeTarget == VelocityFrame.ROBOT
                ? fieldToRobot(heading)
                : robotToField(heading);
    }

    /**
     * Limits the linear speed and angular speed. A non-positive limit leaves that component
     * unclamped, as before.
     */
    public RobotVelocity clamp(LinearVelocity maxLinear, AngularVelocity maxAngular) {
        double x = xMetersPerSecond;
        double y = yMetersPerSecond;
        double maxLinearMetersPerSecond = finiteOrZero(mps(maxLinear));
        double magnitude = Math.hypot(x, y);
        if (maxLinearMetersPerSecond > 0.0 && magnitude > maxLinearMetersPerSecond) {
            double scale = maxLinearMetersPerSecond / magnitude;
            x *= scale;
            y *= scale;
        }
        double maxAngularRadiansPerSecond = finiteOrZero(rps(maxAngular));
        double angular = angularRadiansPerSecond;
        if (maxAngularRadiansPerSecond > 0.0) {
            angular = Math.max(-maxAngularRadiansPerSecond, Math.min(maxAngularRadiansPerSecond, angular));
        }
        return new RobotVelocity(x, y, angular, frame);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof RobotVelocity other)) {
            return false;
        }
        return Double.compare(xMetersPerSecond, other.xMetersPerSecond) == 0
                && Double.compare(yMetersPerSecond, other.yMetersPerSecond) == 0
                && Double.compare(angularRadiansPerSecond, other.angularRadiansPerSecond) == 0
                && frame == other.frame;
    }

    @Override
    public int hashCode() {
        return Objects.hash(xMetersPerSecond, yMetersPerSecond, angularRadiansPerSecond, frame);
    }

    @Override
    public String toString() {
        return "RobotVelocity[x=" + xMetersPerSecond + " m/s, y=" + yMetersPerSecond
                + " m/s, angular=" + angularRadiansPerSecond + " rad/s, frame=" + frame + ']';
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

    private static double mps(LinearVelocity value) {
        return value.in(MetersPerSecond);
    }

    private static double rps(AngularVelocity value) {
        return value.in(RadiansPerSecond);
    }

    private static double radians(Rotation2d heading) {
        return finiteOrZero(Objects.requireNonNull(heading, "heading").getRadians());
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
