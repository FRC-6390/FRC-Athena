package ca.frc6390.athena.runtime.control;

/**
 * Robot-relative or field-relative chassis velocity.
 *
 * @param xMetersPerSecond forward-positive velocity
 * @param yMetersPerSecond left-positive velocity
 * @param angularRadiansPerSecond counter-clockwise angular velocity
 */
public record RobotVelocity(
        double xMetersPerSecond,
        double yMetersPerSecond,
        double angularRadiansPerSecond) {
    public RobotVelocity {
        xMetersPerSecond = finiteOrZero(xMetersPerSecond);
        yMetersPerSecond = finiteOrZero(yMetersPerSecond);
        angularRadiansPerSecond = finiteOrZero(angularRadiansPerSecond);
    }

    /**
     * Creates a stopped velocity.
     *
     * @return zero velocity
     */
    public static RobotVelocity zero() {
        return new RobotVelocity(0.0, 0.0, 0.0);
    }

    /**
     * Converts a field-relative velocity to robot-relative coordinates.
     *
     * @param headingRadians robot heading in radians
     * @return robot-relative velocity
     */
    public RobotVelocity fieldToRobot(double headingRadians) {
        double heading = Double.isFinite(headingRadians) ? headingRadians : 0.0;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new RobotVelocity(
                xMetersPerSecond * cos + yMetersPerSecond * sin,
                -xMetersPerSecond * sin + yMetersPerSecond * cos,
                angularRadiansPerSecond);
    }

    /**
     * Clamps linear and angular magnitude.
     *
     * @param maxLinearMetersPerSecond max linear magnitude, or non-positive for no clamp
     * @param maxAngularRadiansPerSecond max angular magnitude, or non-positive for no clamp
     * @return clamped velocity
     */
    public RobotVelocity clamp(double maxLinearMetersPerSecond, double maxAngularRadiansPerSecond) {
        double x = xMetersPerSecond;
        double y = yMetersPerSecond;
        double maxLinear = finiteOrZero(maxLinearMetersPerSecond);
        double linearMagnitude = Math.hypot(x, y);
        if (maxLinear > 0.0 && linearMagnitude > maxLinear) {
            double scale = maxLinear / linearMagnitude;
            x *= scale;
            y *= scale;
        }
        double maxAngular = finiteOrZero(maxAngularRadiansPerSecond);
        double angular = angularRadiansPerSecond;
        if (maxAngular > 0.0) {
            angular = Math.max(-maxAngular, Math.min(maxAngular, angular));
        }
        return new RobotVelocity(x, y, angular);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
