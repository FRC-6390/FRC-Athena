package ca.frc6390.athena.drivetrain.swerve;

/**
 * Physical module location relative to robot center.
 *
 * @param xMeters forward-positive offset
 * @param yMeters left-positive offset
 */
public record ModuleLocationRef(double xMeters, double yMeters) {
    public ModuleLocationRef {
        if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters)) {
            throw new IllegalArgumentException("Module location must be finite.");
        }
    }

    public static ModuleLocationRef meters(double xMeters, double yMeters) {
        return new ModuleLocationRef(xMeters, yMeters);
    }

    public static ModuleLocationRef inches(double xInches, double yInches) {
        return meters(xInches * 0.0254, yInches * 0.0254);
    }

    public static ModuleLocationRef centimeters(double xCentimeters, double yCentimeters) {
        return meters(xCentimeters / 100.0, yCentimeters / 100.0);
    }
}
