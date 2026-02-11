package ca.frc6390.athena.ctre.imu;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
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
    private boolean cachedConnected = true;

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

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                rollSignal,
                pitchSignal,
                yawSignal,
                velocityXSignal,
                velocityYSignal,
                velocityZSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                accelerationXSignal,
                accelerationYSignal,
                accelerationZSignal);
        pigeon.optimizeBusUtilization(4.0);

        update();
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
    public boolean isConnected() {
        return cachedConnected;
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
