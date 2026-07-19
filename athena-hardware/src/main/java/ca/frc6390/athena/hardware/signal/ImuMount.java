package ca.frc6390.athena.hardware.signal;

import java.util.Objects;

/**
 * Physical orientation of an IMU relative to the robot coordinate frame.
 * Positive X is forward, positive Y is left, and positive Z is up.
 */
public final class ImuMount {
    private static final double EPSILON = 1.0e-9;
    private final double[][] sensorToRobot;

    private ImuMount(double[][] sensorToRobot) {
        this.sensorToRobot = sensorToRobot;
    }

    public static ImuMount identity() {
        return rotationDegrees(0.0, 0.0, 0.0);
    }

    /** Creates a mount from sensor roll, pitch, and yaw relative to the robot. */
    public static ImuMount rotationDegrees(double rollDegrees, double pitchDegrees, double yawDegrees) {
        if (!Double.isFinite(rollDegrees) || !Double.isFinite(pitchDegrees) || !Double.isFinite(yawDegrees)) {
            throw new IllegalArgumentException("IMU mount angles must be finite.");
        }
        return new ImuMount(rotation(
                Math.toRadians(rollDegrees),
                Math.toRadians(pitchDegrees),
                Math.toRadians(yawDegrees)));
    }

    /** Creates a level mount rotated around the robot's up axis. */
    public static ImuMount yawDegrees(double yawDegrees) {
        return rotationDegrees(0.0, 0.0, yawDegrees);
    }

    /** Begins a cardinal mount declaration using the sensor direction that points robot-forward. */
    public static Builder forward(ImuDirection forward) {
        return new Builder(forward);
    }

    Vector rotateVector(double x, double y, double z) {
        return multiply(sensorToRobot, new Vector(x, y, z));
    }

    Orientation robotOrientation(double rollDegrees, double pitchDegrees, double yawDegrees) {
        double[][] fieldFromSensor = rotation(
                Math.toRadians(rollDegrees),
                Math.toRadians(pitchDegrees),
                Math.toRadians(yawDegrees));
        double[][] fieldFromRobot = multiply(fieldFromSensor, transpose(sensorToRobot));
        return orientation(fieldFromRobot);
    }

    Orientation sensorOrientation(double robotRollDegrees, double robotPitchDegrees, double robotYawDegrees) {
        double[][] fieldFromRobot = rotation(
                Math.toRadians(robotRollDegrees),
                Math.toRadians(robotPitchDegrees),
                Math.toRadians(robotYawDegrees));
        return orientation(multiply(fieldFromRobot, sensorToRobot));
    }

    double initialContinuousYaw(double sourceYaw, double sourceAngle, double robotYaw) {
        boolean sourceZIsRobotZ = Math.abs(sensorToRobot[2][0]) < EPSILON
                && Math.abs(sensorToRobot[2][1]) < EPSILON
                && Math.abs(Math.abs(sensorToRobot[2][2]) - 1.0) < EPSILON;
        return sourceZIsRobotZ
                ? robotYaw + sensorToRobot[2][2] * (sourceAngle - sourceYaw)
                : robotYaw;
    }

    /** Completes a cardinal forward/up mount declaration. */
    public static final class Builder {
        private final ImuDirection forward;

        private Builder(ImuDirection forward) {
            this.forward = Objects.requireNonNull(forward, "forward");
        }

        public ImuMount up(ImuDirection up) {
            Objects.requireNonNull(up, "up");
            if (forward.axis() == up.axis()) {
                throw new IllegalArgumentException("IMU forward and up directions must be perpendicular.");
            }
            Vector robotXInSensor = direction(forward);
            Vector robotZInSensor = direction(up);
            Vector robotYInSensor = cross(robotZInSensor, robotXInSensor);
            return new ImuMount(new double[][] {
                    {robotXInSensor.x, robotXInSensor.y, robotXInSensor.z},
                    {robotYInSensor.x, robotYInSensor.y, robotYInSensor.z},
                    {robotZInSensor.x, robotZInSensor.y, robotZInSensor.z}
            });
        }
    }

    record Vector(double x, double y, double z) {
    }

    record Orientation(double rollDegrees, double pitchDegrees, double yawDegrees) {
    }

    private static Vector direction(ImuDirection direction) {
        return switch (direction.axis()) {
            case X -> new Vector(direction.sign(), 0.0, 0.0);
            case Y -> new Vector(0.0, direction.sign(), 0.0);
            case Z -> new Vector(0.0, 0.0, direction.sign());
        };
    }

    private static Vector cross(Vector a, Vector b) {
        return new Vector(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
    }

    private static double[][] rotation(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        return new double[][] {
                {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                {-sp, cp * sr, cp * cr}
        };
    }

    private static Orientation orientation(double[][] value) {
        double pitch = Math.asin(clamp(-value[2][0], -1.0, 1.0));
        double roll;
        double yaw;
        if (Math.abs(Math.cos(pitch)) > EPSILON) {
            roll = Math.atan2(value[2][1], value[2][2]);
            yaw = Math.atan2(value[1][0], value[0][0]);
        } else {
            roll = 0.0;
            yaw = Math.atan2(-value[0][1], value[1][1]);
        }
        return new Orientation(Math.toDegrees(roll), Math.toDegrees(pitch), Math.toDegrees(yaw));
    }

    private static Vector multiply(double[][] matrix, Vector vector) {
        return new Vector(
                matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
                matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
                matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z);
    }

    private static double[][] multiply(double[][] a, double[][] b) {
        double[][] result = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                result[row][column] = a[row][0] * b[0][column]
                        + a[row][1] * b[1][column]
                        + a[row][2] * b[2][column];
            }
        }
        return result;
    }

    private static double[][] transpose(double[][] value) {
        return new double[][] {
                {value[0][0], value[1][0], value[2][0]},
                {value[0][1], value[1][1], value[2][1]},
                {value[0][2], value[1][2], value[2][2]}
        };
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
