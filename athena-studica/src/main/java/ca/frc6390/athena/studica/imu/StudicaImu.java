package ca.frc6390.athena.studica.imu;

import java.util.ArrayDeque;
import java.util.Deque;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Studica (NavX) IMU wrapper for the new vendordep system.
 */
public class StudicaImu implements Imu {
    private static final double DEFAULT_MAX_SPEED_WINDOW_SECONDS = 5.0;
    private static final double MIN_MAX_SPEED_WINDOW_SECONDS = 0.02;

    private final ImuConfig config;
    private final AHRS navx;
    private final Deque<SpeedSample> maxSpeedSamples = new ArrayDeque<>();
    private double maxSpeedWindowSeconds = DEFAULT_MAX_SPEED_WINDOW_SECONDS;
    private double maxLinearSpeed = 0.0;
    private double maxRadialSpeed = 0.0;

    private static final class SpeedSample {
        private final double timestampSeconds;
        private final double linearSpeed;
        private final double radialSpeed;

        private SpeedSample(double timestampSeconds, double linearSpeed, double radialSpeed) {
            this.timestampSeconds = timestampSeconds;
            this.linearSpeed = linearSpeed;
            this.radialSpeed = radialSpeed;
        }
    }

    public StudicaImu(AHRS navx, ImuConfig config) {
        this.navx = navx;
        this.config = config;
    }

    public static StudicaImu fromConfig(ImuConfig config) {
        if (config == null || !(config.type() instanceof StudicaImuType)) {
            throw new IllegalArgumentException("Studica IMU config required");
        }

        AHRS navx = new AHRS(NavXComType.kMXP_SPI);
        return new StudicaImu(navx, config);
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(navx.getRoll());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(navx.getPitch());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(navx.getYaw());
    }

    @Override
    public Rotation2d getVelocityZ() {
        return Rotation2d.fromDegrees(navx.getRate());
    }

    @Override
    public double getXSpeedMetersPerSecond() {
        return navx.getVelocityX();
    }

    @Override
    public double getYSpeedMetersPerSecond() {
        return navx.getVelocityY();
    }

    @Override
    public double getAccelerationX() {
        return navx.getWorldLinearAccelX();
    }

    @Override
    public double getAccelerationY() {
        return navx.getWorldLinearAccelY();
    }

    @Override
    public double getAccelerationZ() {
        return navx.getWorldLinearAccelZ();
    }

    @Override
    public boolean isConnected() {
        return navx.isConnected();
    }

    @Override
    public double getMaxLinearSpeed() {
        return maxLinearSpeed;
    }

    @Override
    public double getMaxRadialSpeed() {
        return maxRadialSpeed;
    }

    @Override
    public double getMaxSpeedWindowSeconds() {
        return maxSpeedWindowSeconds;
    }

    @Override
    public void setMaxSpeedWindowSeconds(double windowSeconds) {
        if (!Double.isFinite(windowSeconds)) {
            return;
        }
        maxSpeedWindowSeconds = Math.max(windowSeconds, MIN_MAX_SPEED_WINDOW_SECONDS);
        pruneMaxSpeedSamples(Timer.getFPGATimestamp());
        recomputeMaxSpeeds();
    }

    @Override
    public void resetMaxSpeedWindow() {
        maxSpeedSamples.clear();
        maxLinearSpeed = 0.0;
        maxRadialSpeed = 0.0;
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.hardware().inverted(inverted);
        }
    }

    @Override
    public void update() {
        updateMaxSpeedTracking();
    }

    @Override
    public ImuConfig getConfig() {
        return config;
    }

    private void updateMaxSpeedTracking() {
        if (!Double.isFinite(maxSpeedWindowSeconds) || maxSpeedWindowSeconds <= 0.0) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        double linearSpeed = getMovementSpeedMetersPerSecond();
        double radialSpeed = Math.abs(getThetaSpeedRadiansPerSecond());
        if (!Double.isFinite(linearSpeed)) {
            linearSpeed = 0.0;
        }
        if (!Double.isFinite(radialSpeed)) {
            radialSpeed = 0.0;
        }
        maxSpeedSamples.addLast(new SpeedSample(now, linearSpeed, radialSpeed));
        pruneMaxSpeedSamples(now);
        recomputeMaxSpeeds();
    }

    private void pruneMaxSpeedSamples(double nowSeconds) {
        double cutoff = nowSeconds - maxSpeedWindowSeconds;
        while (!maxSpeedSamples.isEmpty() && maxSpeedSamples.peekFirst().timestampSeconds < cutoff) {
            maxSpeedSamples.removeFirst();
        }
    }

    private void recomputeMaxSpeeds() {
        double linear = 0.0;
        double radial = 0.0;
        for (SpeedSample sample : maxSpeedSamples) {
            if (sample.linearSpeed > linear) {
                linear = sample.linearSpeed;
            }
            if (sample.radialSpeed > radial) {
                radial = sample.radialSpeed;
            }
        }
        maxLinearSpeed = linear;
        maxRadialSpeed = radial;
    }

}
