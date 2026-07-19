package ca.frc6390.athena.hardware.signal;

import java.util.Objects;

/**
 * Signed axis mapping used to normalize an IMU vendor's coordinate convention.
 * Output X/Y/Z correspond to roll/pitch/yaw, angular rate, and acceleration axes.
 */
public record ImuTransform(ImuDirection x, ImuDirection y, ImuDirection z) {
    public ImuTransform {
        Objects.requireNonNull(x, "x");
        Objects.requireNonNull(y, "y");
        Objects.requireNonNull(z, "z");
        if (x.axis() == y.axis() || x.axis() == z.axis() || y.axis() == z.axis()) {
            throw new IllegalArgumentException("An IMU transform must use each source axis exactly once.");
        }
    }

    public static ImuTransform identity() {
        return new ImuTransform(
                ImuDirection.POSITIVE_X,
                ImuDirection.POSITIVE_Y,
                ImuDirection.POSITIVE_Z);
    }

    public ImuTransform rollInverted() {
        return new ImuTransform(x.opposite(), y, z);
    }

    public ImuTransform pitchInverted() {
        return new ImuTransform(x, y.opposite(), z);
    }

    public ImuTransform yawInverted() {
        return new ImuTransform(x, y, z.opposite());
    }

    public ImuTransform inverted(ImuAxis axis) {
        Objects.requireNonNull(axis, "axis");
        return switch (axis) {
            case X -> rollInverted();
            case Y -> pitchInverted();
            case Z -> yawInverted();
        };
    }

    public ImuTransform swapRollAndPitch() {
        return new ImuTransform(y, x, z);
    }

    public ImuTransform swapPitchAndYaw() {
        return new ImuTransform(x, z, y);
    }

    public ImuTransform swapRollAndYaw() {
        return new ImuTransform(z, y, x);
    }

    Vector apply(double sourceX, double sourceY, double sourceZ) {
        return new Vector(
                x.component(sourceX, sourceY, sourceZ),
                y.component(sourceX, sourceY, sourceZ),
                z.component(sourceX, sourceY, sourceZ));
    }

    boolean preservesYawAxis() {
        return z.axis() == ImuAxis.Z;
    }

    double sourceYaw(double transformedYaw) {
        if (!preservesYawAxis()) {
            throw new UnsupportedOperationException(
                    "Cannot set yaw through an IMU transform that maps yaw from another axis.");
        }
        return transformedYaw * z.sign();
    }

    record Vector(double x, double y, double z) {
    }
}
