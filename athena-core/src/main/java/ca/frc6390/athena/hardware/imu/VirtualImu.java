package ca.frc6390.athena.hardware.imu;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Decorates a concrete {@link Imu} with virtual-axis support to preserve legacy heading helpers
 * while routing all sensor reads through the new vendordep-backed IMU implementations.
 */
public class VirtualImu implements Imu {
    private static final double DEFAULT_MAX_SPEED_WINDOW_SECONDS = 5.0;
    private static final double MIN_MAX_SPEED_WINDOW_SECONDS = 0.02;

    private final Imu delegate;
    private final Map<String, VirtualAxis> virtualAxes = new HashMap<>();
    private boolean useSimulatedReadings = false;
    private boolean inverted = false;
    private Rotation2d simRoll = new Rotation2d();
    private Rotation2d simPitch = new Rotation2d();
    private Rotation2d simYaw = new Rotation2d();
    private Rotation2d simVelX = new Rotation2d();
    private Rotation2d simVelY = new Rotation2d();
    private Rotation2d simVelZ = new Rotation2d();
    private final Deque<SpeedSample> maxSpeedSamples = new ArrayDeque<>();
    private double maxSpeedWindowSeconds = DEFAULT_MAX_SPEED_WINDOW_SECONDS;
    private double maxLinearSpeed = 0.0;
    private double maxRadialSpeed = 0.0;
    private double cachedAngularAccelerationZ = 0.0;
    private double lastVelZRadians = Double.NaN;
    private double lastUpdateSeconds = Double.NaN;

    private static class VirtualAxis {
        private final Supplier<Rotation2d> supplier;
        private Rotation2d offset = new Rotation2d();

        VirtualAxis(Supplier<Rotation2d> supplier) {
            this.supplier = supplier;
        }

        Rotation2d get() {
            return supplier.get().minus(offset);
        }

        void set(Rotation2d value) {
            offset = supplier.get().minus(value);
        }

        void setOffset(Rotation2d value) {
            offset = value;
        }
    }

    public VirtualImu(Imu delegate) {
        this.delegate = delegate;
        addVirtualAxis("driver", this::getYaw);
        ImuConfig config = delegate.getConfig();
        this.inverted = config != null && config.inverted();
    }

    private static class SpeedSample {
        private final double timestampSeconds;
        private final double linearSpeed;
        private final double radialSpeed;

        private SpeedSample(double timestampSeconds, double linearSpeed, double radialSpeed) {
            this.timestampSeconds = timestampSeconds;
            this.linearSpeed = linearSpeed;
            this.radialSpeed = radialSpeed;
        }
    }

    @Override
    public Rotation2d getRoll() {
        return useSimulatedReadings ? applySimInversion(simRoll) : delegate.getRoll();
    }

    @Override
    public Rotation2d getPitch() {
        return useSimulatedReadings ? applySimInversion(simPitch) : delegate.getPitch();
    }

    @Override
    public Rotation2d getYaw() {
        return useSimulatedReadings ? applySimInversion(simYaw) : delegate.getYaw();
    }

    @Override
    public Rotation2d getVelocityX() {
        return useSimulatedReadings ? applySimInversion(simVelX) : delegate.getVelocityX();
    }

    @Override
    public Rotation2d getVelocityY() {
        return useSimulatedReadings ? applySimInversion(simVelY) : delegate.getVelocityY();
    }

    @Override
    public Rotation2d getVelocityZ() {
        return useSimulatedReadings ? applySimInversion(simVelZ) : delegate.getVelocityZ();
    }

    @Override
    public double getXSpeedMetersPerSecond() {
        return useSimulatedReadings ? 0.0 : delegate.getXSpeedMetersPerSecond();
    }

    @Override
    public double getYSpeedMetersPerSecond() {
        return useSimulatedReadings ? 0.0 : delegate.getYSpeedMetersPerSecond();
    }

    @Override
    public double getAngularAccelerationZRadiansPerSecondSquared() {
        return cachedAngularAccelerationZ;
    }

    @Override
    public double getAccelerationX() {
        return useSimulatedReadings ? 0.0 : delegate.getAccelerationX();
    }

    @Override
    public double getAccelerationY() {
        return useSimulatedReadings ? 0.0 : delegate.getAccelerationY();
    }

    @Override
    public double getAccelerationZ() {
        return useSimulatedReadings ? 0.0 : delegate.getAccelerationZ();
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        delegate.setInverted(inverted);
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }

    @Override
    public boolean isConnected() {
        return delegate.isConnected();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        setVirtualAxis("driver", yaw);
    }

    @Override
    public void setYaw(double yawDegrees) {
        setYaw(Rotation2d.fromDegrees(yawDegrees));
    }

    @Override
    public void addVirtualAxis(String name, Supplier<Rotation2d> supplier) {
        virtualAxes.put(name, new VirtualAxis(supplier));
    }

    @Override
    public Rotation2d getVirtualAxis(String name) {
        VirtualAxis axis = virtualAxes.get(name);
        return axis != null ? axis.get() : new Rotation2d();
    }

    @Override
    public void setVirtualAxis(String name, Rotation2d value) {
        VirtualAxis axis = virtualAxes.get(name);
        if (axis != null) {
            axis.set(value);
        }
    }

    public void setVirtualOffset(String name, Rotation2d value) {
        VirtualAxis axis = virtualAxes.get(name);
        if (axis != null) {
            axis.setOffset(value);
        }
    }

    private Rotation2d applySimInversion(Rotation2d value) {
        if (!inverted) {
            return value;
        }
        return Rotation2d.fromRadians(-value.getRadians());
    }

    @Override
    public void update() {
        if (!useSimulatedReadings) {
            delegate.update();
        }
        updateAngularAcceleration();
        updateMaxSpeedTracking();
    }

    @Override
    public void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {
        setSimulatedReadings(yaw, new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d(), angularVelocityZ);
    }

    @Override
    public void setSimulatedReadings(Rotation2d yaw, Rotation2d pitch, Rotation2d roll,
                                     Rotation2d velX, Rotation2d velY, Rotation2d velZ) {
        simYaw = yaw;
        simPitch = pitch;
        simRoll = roll;
        simVelX = velX;
        simVelY = velY;
        simVelZ = velZ;
        useSimulatedReadings = true;
    }

    @Override
    public void disableSimulatedReadings() {
        useSimulatedReadings = false;
    }

    @Override
    public ImuConfig getConfig() {
        return delegate.getConfig();
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

    private void updateMaxSpeedTracking() {
        if (!Double.isFinite(maxSpeedWindowSeconds) || maxSpeedWindowSeconds <= 0.0) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        double linearSpeed = computeLinearSpeed();
        double radialSpeed = computeRadialSpeed();
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

    private double computeLinearSpeed() {
        double speed = getMovementSpeedMetersPerSecond();
        return Double.isFinite(speed) ? speed : 0.0;
    }

    private double computeRadialSpeed() {
        double omega = Math.abs(getThetaSpeedRadiansPerSecond());
        return Double.isFinite(omega) ? omega : 0.0;
    }

    private void updateAngularAcceleration() {
        double now = Timer.getFPGATimestamp();
        Rotation2d velZ = getVelocityZ();
        double currentVel = velZ != null ? velZ.getRadians() : 0.0;
        if (Double.isNaN(lastUpdateSeconds) || !Double.isFinite(lastUpdateSeconds)) {
            cachedAngularAccelerationZ = 0.0;
        } else {
            double dt = now - lastUpdateSeconds;
            if (dt > 0.0 && Double.isFinite(dt)) {
                double accel = (currentVel - lastVelZRadians) / dt;
                cachedAngularAccelerationZ = Double.isFinite(accel) ? accel : 0.0;
            } else {
                cachedAngularAccelerationZ = 0.0;
            }
        }
        lastUpdateSeconds = now;
        lastVelZRadians = currentVel;
    }
}
