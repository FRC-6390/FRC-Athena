package ca.frc6390.athena.vision.device;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Robot-relative camera mount pose.
 *
 * @param x forward-positive position
 * @param y left-positive position
 * @param z up-positive position
 * @param yaw yaw offset
 * @param pitch pitch offset
 * @param roll roll offset
 */
public record CameraMountPose(
        Distance x,
        Distance y,
        Distance z,
        Angle yaw,
        Angle pitch,
        Angle roll) {
    /**
     * Returns a zero-offset camera pose.
     *
     * @return identity mount pose
     */
    public static CameraMountPose identity() {
        return new CameraMountPose(
                Meters.zero(),
                Meters.zero(),
                Meters.zero(),
                Degrees.zero(),
                Degrees.zero(),
                Degrees.zero());
    }

    /**
     * Returns whether every pose value is finite.
     *
     * @return true if finite
     */
    public boolean isFinite() {
        return Double.isFinite(x.in(Meters))
                && Double.isFinite(y.in(Meters))
                && Double.isFinite(z.in(Meters))
                && Double.isFinite(yaw.in(Degrees))
                && Double.isFinite(pitch.in(Degrees))
                && Double.isFinite(roll.in(Degrees));
    }
}
