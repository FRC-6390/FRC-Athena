package ca.frc6390.athena.ctre.imu;

import com.ctre.phoenix6.CANBus;
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
    public CtreImu(Pigeon2 pigeon, ImuConfig config) {
        this.pigeon = pigeon;
        this.config = config;
    }

    public static CtreImu fromConfig(ImuConfig config) {
        if (config == null || !(config.type instanceof CtreImuType)) {
            throw new IllegalArgumentException("CTRE IMU config required");
        }

        Pigeon2 pigeon = new Pigeon2(config.id, resolveCanBus(config.canbus));
        return new CtreImu(pigeon, config);
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(pigeon.getRoll(true).getValueAsDouble());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pigeon.getPitch(true).getValueAsDouble());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw(true).getValueAsDouble());
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.inverted = inverted;
        }
    }

    @Override
    public ImuConfig getConfig() {
        return config;
    }

    private static CANBus resolveCanBus(String canbus) {
        if (canbus == null || canbus.isBlank()) {
            return new CANBus();
        }
        return new CANBus(canbus);
    }
}
