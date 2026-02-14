package ca.frc6390.athena.ctre.imu;

import java.util.ArrayDeque;
import java.util.Deque;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import ca.frc6390.athena.ctre.CtreCanBusRegistry;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * CTRE IMU wrapper for the new vendordep system.
 */
public class CtreImu implements Imu {
    private static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665;
    private static final double ACCELERATION_DEADBAND_G = 0.02;
    private static final double STATIONARY_VELOCITY_DECAY_PER_SECOND = 0.85;
    private static final double DEFAULT_MAX_SPEED_WINDOW_SECONDS = 5.0;
    private static final double MIN_MAX_SPEED_WINDOW_SECONDS = 0.02;

    private final ImuConfig config;
    private final Pigeon2 pigeon;
    private final StatusSignal<?> rollSignal;
    private final StatusSignal<?> pitchSignal;
    private final StatusSignal<?> yawSignal;
    private final StatusSignal<?> velocityXSignal;
    private final StatusSignal<?> velocityYSignal;
    private final StatusSignal<?> velocityZSignal;
    private final StatusSignal<?> accelerationXSignal;
    private final StatusSignal<?> accelerationYSignal;
    private final StatusSignal<?> accelerationZSignal;
    private final BaseStatusSignal[] refreshSignals;
    private double cachedRollDegrees;
    private double cachedPitchDegrees;
    private double cachedYawDegrees;
    private double cachedVelocityXDegreesPerSecond;
    private double cachedVelocityYDegreesPerSecond;
    private double cachedVelocityZDegreesPerSecond;
    private double cachedAccelerationX;
    private double cachedAccelerationY;
    private double cachedAccelerationZ;
    private double cachedXSpeedMetersPerSecond;
    private double cachedYSpeedMetersPerSecond;
    private double lastUpdateSeconds = Double.NaN;
    private final Deque<SpeedSample> maxSpeedSamples = new ArrayDeque<>();
    private double maxSpeedWindowSeconds = DEFAULT_MAX_SPEED_WINDOW_SECONDS;
    private double maxLinearSpeed = 0.0;
    private double maxRadialSpeed = 0.0;
    private boolean cachedConnected = true;

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

    public CtreImu(Pigeon2 pigeon, ImuConfig config) {
        this.pigeon = pigeon;
        this.config = config;
        this.rollSignal = pigeon.getRoll(false).clone();
        this.pitchSignal = pigeon.getPitch(false).clone();
        this.yawSignal = pigeon.getYaw(false).clone();
        this.velocityXSignal = pigeon.getAngularVelocityXDevice(false).clone();
        this.velocityYSignal = pigeon.getAngularVelocityYDevice(false).clone();
        this.velocityZSignal = pigeon.getAngularVelocityZDevice(false).clone();
        this.accelerationXSignal = pigeon.getAccelerationX(false).clone();
        this.accelerationYSignal = pigeon.getAccelerationY(false).clone();
        this.accelerationZSignal = pigeon.getAccelerationZ(false).clone();
        this.refreshSignals = new BaseStatusSignal[] {
                rollSignal,
                pitchSignal,
                yawSignal,
                velocityXSignal,
                velocityYSignal,
                velocityZSignal,
                accelerationXSignal,
                accelerationYSignal,
                accelerationZSignal
        };

        rollSignal.setUpdateFrequency(100.0, 0.0);
        pitchSignal.setUpdateFrequency(100.0, 0.0);
        yawSignal.setUpdateFrequency(100.0, 0.0);
        velocityXSignal.setUpdateFrequency(100.0, 0.0);
        velocityYSignal.setUpdateFrequency(100.0, 0.0);
        velocityZSignal.setUpdateFrequency(100.0, 0.0);
        accelerationXSignal.setUpdateFrequency(50.0, 0.0);
        accelerationYSignal.setUpdateFrequency(50.0, 0.0);
        accelerationZSignal.setUpdateFrequency(50.0, 0.0);
        pigeon.optimizeBusUtilization(4.0, 0.0);
    }

    public static CtreImu fromConfig(ImuConfig config) {
        if (config == null || !(config.type() instanceof CtreImuType)) {
            throw new IllegalArgumentException("CTRE IMU config required");
        }

        Pigeon2 pigeon = new Pigeon2(config.id(), resolveCanBus(config.canbus()));
        return new CtreImu(pigeon, config);
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(cachedRollDegrees);
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(cachedPitchDegrees);
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(cachedYawDegrees);
    }

    @Override
    public Rotation2d getVelocityZ() {
        return Rotation2d.fromDegrees(cachedVelocityZDegreesPerSecond);
    }

    @Override
    public Rotation2d getVelocityX() {
        return Rotation2d.fromDegrees(cachedVelocityXDegreesPerSecond);
    }   

    @Override
    public Rotation2d getVelocityY() {
        return Rotation2d.fromDegrees(cachedVelocityYDegreesPerSecond);
    }

    @Override
    public double getAccelerationX() {
        return cachedAccelerationX;
    }

    @Override
    public double getAccelerationY() {
        return cachedAccelerationY;
    }

    @Override
    public double getAccelerationZ() {
        return cachedAccelerationZ;
    }

    @Override
    public double getXSpeedMetersPerSecond() {
        return cachedXSpeedMetersPerSecond;
    }

    @Override
    public double getYSpeedMetersPerSecond() {
        return cachedYSpeedMetersPerSecond;
    }

    @Override
    public boolean isConnected() {
        return cachedConnected;
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
    public void update() {
        BaseStatusSignal.refreshAll(refreshSignals);
        cachedRollDegrees = rollSignal.getValueAsDouble();
        cachedPitchDegrees = pitchSignal.getValueAsDouble();
        cachedYawDegrees = yawSignal.getValueAsDouble();
        cachedVelocityXDegreesPerSecond = velocityXSignal.getValueAsDouble();
        cachedVelocityYDegreesPerSecond = velocityYSignal.getValueAsDouble();
        cachedVelocityZDegreesPerSecond = velocityZSignal.getValueAsDouble();
        cachedAccelerationX = accelerationXSignal.getValueAsDouble();
        cachedAccelerationY = accelerationYSignal.getValueAsDouble();
        cachedAccelerationZ = accelerationZSignal.getValueAsDouble();
        cachedConnected = pigeon.isConnected();
        updateLinearVelocityEstimate();
        updateMaxSpeedTracking();
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.hardware().inverted(inverted);
        }
    }

    @Override
    public ImuConfig getConfig() {
        return config;
    }

    private static CANBus resolveCanBus(String canbus) {
        return CtreCanBusRegistry.resolve(canbus);
    }

    private void updateLinearVelocityEstimate() {
        double nowSeconds = Timer.getFPGATimestamp();
        if (!cachedConnected) {
            cachedXSpeedMetersPerSecond = 0.0;
            cachedYSpeedMetersPerSecond = 0.0;
            lastUpdateSeconds = nowSeconds;
            return;
        }
        if (!Double.isFinite(lastUpdateSeconds)) {
            lastUpdateSeconds = nowSeconds;
            return;
        }

        double dt = nowSeconds - lastUpdateSeconds;
        lastUpdateSeconds = nowSeconds;
        if (!Double.isFinite(dt) || dt <= 1e-6 || dt > 0.25) {
            return;
        }

        double axG = sanitizeAccelerationG(cachedAccelerationX);
        double ayG = sanitizeAccelerationG(cachedAccelerationY);
        cachedXSpeedMetersPerSecond += axG * GRAVITY_METERS_PER_SECOND_SQUARED * dt;
        cachedYSpeedMetersPerSecond += ayG * GRAVITY_METERS_PER_SECOND_SQUARED * dt;

        if (Math.abs(axG) < ACCELERATION_DEADBAND_G && Math.abs(ayG) < ACCELERATION_DEADBAND_G) {
            double decay = Math.exp(-STATIONARY_VELOCITY_DECAY_PER_SECOND * dt);
            cachedXSpeedMetersPerSecond *= decay;
            cachedYSpeedMetersPerSecond *= decay;
        }
    }

    private static double sanitizeAccelerationG(double accelG) {
        if (!Double.isFinite(accelG)) {
            return 0.0;
        }
        return Math.abs(accelG) < ACCELERATION_DEADBAND_G ? 0.0 : accelG;
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
        double maxLinear = 0.0;
        double maxRadial = 0.0;
        for (SpeedSample sample : maxSpeedSamples) {
            if (sample.linearSpeed > maxLinear) {
                maxLinear = sample.linearSpeed;
            }
            if (sample.radialSpeed > maxRadial) {
                maxRadial = sample.radialSpeed;
            }
        }
        maxLinearSpeed = maxLinear;
        maxRadialSpeed = maxRadial;
    }
}
