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
        this.inverted = config.inverted;
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
        return Rotation2d.fromDegrees(direction() * navx.getRoll());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(direction() * navx.getPitch());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(direction() * navx.getYaw());
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public ImuConfig getConfig() {
        return config;
    }

    private double direction() {
        return inverted ? -1.0 : 1.0;
    }
}
