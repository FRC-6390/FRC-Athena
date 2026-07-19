package ca.frc6390.athena.hardware.signal;

/** A signed IMU axis used to describe wiring conventions and physical mounting. */
public enum ImuDirection {
    POSITIVE_X(ImuAxis.X, 1.0),
    NEGATIVE_X(ImuAxis.X, -1.0),
    POSITIVE_Y(ImuAxis.Y, 1.0),
    NEGATIVE_Y(ImuAxis.Y, -1.0),
    POSITIVE_Z(ImuAxis.Z, 1.0),
    NEGATIVE_Z(ImuAxis.Z, -1.0);

    private final ImuAxis axis;
    private final double sign;

    ImuDirection(ImuAxis axis, double sign) {
        this.axis = axis;
        this.sign = sign;
    }

    public ImuAxis axis() {
        return axis;
    }

    public double sign() {
        return sign;
    }

    public ImuDirection opposite() {
        return values()[ordinal() ^ 1];
    }

    double component(double x, double y, double z) {
        return sign * switch (axis) {
            case X -> x;
            case Y -> y;
            case Z -> z;
        };
    }
}
