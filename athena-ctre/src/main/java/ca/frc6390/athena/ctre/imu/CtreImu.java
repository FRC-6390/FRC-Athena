package ca.frc6390.athena.ctre.imu;

import com.ctre.phoenix6.hardware.Pigeon2;

import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * CTRE IMU wrapper for the new vendordep system.
 */
public class CtreImu implements Imu {
    private final ImuConfig config;
    private final Pigeon2 pigeon;
    private boolean inverted;

    public CtreImu(Pigeon2 pigeon, ImuConfig config) {
        this.pigeon = pigeon;
        this.config = config;
        this.inverted = config.inverted;
    }

    public static CtreImu fromConfig(ImuConfig config) {
        if (config == null || !(config.type instanceof CtreImuType)) {
            throw new IllegalArgumentException("CTRE IMU config required");
        }

        Pigeon2 pigeon = new Pigeon2(config.id, config.canbus);
        return new CtreImu(pigeon, config);
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(direction() * pigeon.getRoll(true).getValueAsDouble());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(direction() * pigeon.getPitch(true).getValueAsDouble());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(direction() * pigeon.getYaw(true).getValueAsDouble());
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
