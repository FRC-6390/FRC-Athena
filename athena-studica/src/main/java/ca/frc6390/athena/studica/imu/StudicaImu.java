package ca.frc6390.athena.studica.imu;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Studica (NavX) IMU wrapper for the new vendordep system.
 */
public class StudicaImu implements Imu {
    private final ImuConfig config;
    private final AHRS navx;
    private boolean inverted;
    public StudicaImu(AHRS navx, ImuConfig config) {
        this.navx = navx;
        this.config = config;
        this.inverted = config != null && config.inverted;
    }

    public static StudicaImu fromConfig(ImuConfig config) {
        if (config == null || !(config.type instanceof StudicaImuType)) {
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
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        if (config != null) {
            config.inverted = inverted;
        }
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }

    @Override
    public boolean isConnected() {
        return navx.isConnected();
    }

    @Override
    public Rotation2d getVelocityX() {
        return Rotation2d.fromDegrees(navx.getRawGyroX());
    }

    @Override
    public Rotation2d getVelocityY() {
        return Rotation2d.fromDegrees(navx.getRawGyroY());
    }

    @Override
    public Rotation2d getVelocityZ() {
        return Rotation2d.fromDegrees(navx.getRawGyroZ());
    }

    @Override
    public double getAccelX() {
        return navx.getWorldLinearAccelX();
    }

    @Override
    public double getAccelY() {
        return navx.getWorldLinearAccelY();
    }

    @Override
    public double getAccelZ() {
        return navx.getWorldLinearAccelZ();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        if (yaw == null) {
            return;
        }
        double adjustment = yaw.getDegrees() - navx.getYaw();
        navx.setAngleAdjustment(adjustment);
    }

    @Override
    public ImuConfig getConfig() {
        return config;
    }

}
