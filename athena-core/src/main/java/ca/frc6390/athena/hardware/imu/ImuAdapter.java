package ca.frc6390.athena.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Applies shared IMU configuration (inversion) on top of vendor-specific IMUs.
 */
public class ImuAdapter implements Imu {
    private final Imu raw;
    private final ImuConfig config;
    private boolean inverted;
    private Rotation2d cachedRoll = new Rotation2d();
    private Rotation2d cachedPitch = new Rotation2d();
    private Rotation2d cachedYaw = new Rotation2d();
    private Rotation2d cachedVelX = new Rotation2d();
    private Rotation2d cachedVelY = new Rotation2d();
    private Rotation2d cachedVelZ = new Rotation2d();
    private double cachedXSpeedMetersPerSecond;
    private double cachedYSpeedMetersPerSecond;
    private double cachedAngularAccelerationZ;
    private double cachedAccelerationX;
    private double cachedAccelerationY;
    private double cachedAccelerationZ;
    private boolean cachedConnected = true;
    private double lastVelZRadians = Double.NaN;
    private double lastUpdateSeconds = Double.NaN;

    public ImuAdapter(Imu raw, ImuConfig config) {
        this.raw = raw;
        this.config = config != null ? config : raw.getConfig();
        this.inverted = this.config != null && this.config.inverted();
    }

    public static Imu wrap(Imu raw, ImuConfig config) {
        if (raw == null) {
            return null;
        }
        if (raw instanceof ImuAdapter) {
            return raw;
        }
        return new ImuAdapter(raw, config);
    }

    @Override
    public Rotation2d getRoll() {
        return cachedRoll;
    }

    @Override
    public Rotation2d getPitch() {
        return cachedPitch;
    }

    @Override
    public Rotation2d getYaw() {
        return cachedYaw;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        if (config != null) {
            config.setInverted(inverted);
        }
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }

    @Override
    public boolean isConnected() {
        return cachedConnected;
    }

    @Override
    public Rotation2d getVelocityX() {
        return cachedVelX;
    }

    @Override
    public Rotation2d getVelocityY() {
        return cachedVelY;
    }

    @Override
    public Rotation2d getVelocityZ() {
        return cachedVelZ;
    }

    @Override
    public double getAngularAccelerationZRadiansPerSecondSquared() {
        return cachedAngularAccelerationZ;
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
    public double getMaxLinearSpeed() {
        return raw.getMaxLinearSpeed();
    }

    @Override
    public double getMaxRadialSpeed() {
        return raw.getMaxRadialSpeed();
    }

    @Override
    public double getMaxSpeedWindowSeconds() {
        return raw.getMaxSpeedWindowSeconds();
    }

    @Override
    public void setMaxSpeedWindowSeconds(double windowSeconds) {
        raw.setMaxSpeedWindowSeconds(windowSeconds);
    }

    @Override
    public void resetMaxSpeedWindow() {
        raw.resetMaxSpeedWindow();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        raw.setYaw(inverted ? Rotation2d.fromRadians(-yaw.getRadians()) : yaw);
    }

    @Override
    public void setYaw(double yawDegrees) {
        setYaw(Rotation2d.fromDegrees(yawDegrees));
    }

    @Override
    public void addVirtualAxis(String name, java.util.function.Supplier<Rotation2d> supplier) {
        raw.addVirtualAxis(name, supplier);
    }

    @Override
    public Rotation2d getVirtualAxis(String name) {
        return raw.getVirtualAxis(name);
    }

    @Override
    public void setVirtualAxis(String name, Rotation2d value) {
        raw.setVirtualAxis(name, value);
    }

    @Override
    public void update() {
        raw.update();
        cachedRoll = invertRotation(raw.getRoll());
        cachedPitch = invertRotation(raw.getPitch());
        cachedYaw = invertRotation(raw.getYaw());
        cachedVelX = invertRotation(raw.getVelocityX());
        cachedVelY = invertRotation(raw.getVelocityY());
        cachedVelZ = invertRotation(raw.getVelocityZ());
        double xMetersPerSecond = raw.getXSpeedMetersPerSecond();
        double yMetersPerSecond = raw.getYSpeedMetersPerSecond();
        cachedXSpeedMetersPerSecond = inverted ? -xMetersPerSecond : xMetersPerSecond;
        cachedYSpeedMetersPerSecond = inverted ? -yMetersPerSecond : yMetersPerSecond;
        updateAngularAcceleration();
        cachedAccelerationX = inverted ? -raw.getAccelerationX() : raw.getAccelerationX();
        cachedAccelerationY = inverted ? -raw.getAccelerationY() : raw.getAccelerationY();
        cachedAccelerationZ = inverted ? -raw.getAccelerationZ() : raw.getAccelerationZ();
        cachedConnected = raw.isConnected();
    }

    @Override
    public void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {
        setSimulatedReadings(yaw, new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d(), angularVelocityZ);
    }

    @Override
    public void setSimulatedReadings(Rotation2d yaw, Rotation2d pitch, Rotation2d roll,
                                     Rotation2d velX, Rotation2d velY, Rotation2d velZ) {
        if (inverted) {
            raw.setSimulatedReadings(invertRotation(yaw), invertRotation(pitch), invertRotation(roll),
                    invertRotation(velX), invertRotation(velY), invertRotation(velZ));
        } else {
            raw.setSimulatedReadings(yaw, pitch, roll, velX, velY, velZ);
        }
    }

    @Override
    public void disableSimulatedReadings() {
        raw.disableSimulatedReadings();
    }

    @Override
    public ImuConfig getConfig() {
        return config;
    }

    private Rotation2d invertRotation(Rotation2d value) {
        if (!inverted) {
            return value;
        }
        return Rotation2d.fromRadians(-value.getRadians());
    }

    private void updateAngularAcceleration() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double currentVel = cachedVelZ != null ? cachedVelZ.getRadians() : 0.0;
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
