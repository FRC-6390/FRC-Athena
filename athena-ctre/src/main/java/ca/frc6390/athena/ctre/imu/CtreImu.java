package ca.frc6390.athena.ctre.imu;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

/**
 * CTRE IMU wrapper for the new vendordep system.
 */
public class CtreImu implements Imu {
    private final ImuConfig config;
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<AngularVelocity> velXSignal;
    private final StatusSignal<AngularVelocity> velYSignal;
    private final StatusSignal<AngularVelocity> velZSignal;
    private final StatusSignal<LinearAcceleration> accelXSignal;
    private final StatusSignal<LinearAcceleration> accelYSignal;
    private final StatusSignal<LinearAcceleration> accelZSignal;
    public CtreImu(Pigeon2 pigeon, ImuConfig config) {
        this.pigeon = pigeon;
        this.config = config;
        this.yawSignal = pigeon.getYaw(false);
        this.pitchSignal = pigeon.getPitch(false);
        this.rollSignal = pigeon.getRoll(false);
        this.velXSignal = pigeon.getAngularVelocityXWorld(false);
        this.velYSignal = pigeon.getAngularVelocityYWorld(false);
        this.velZSignal = pigeon.getAngularVelocityZWorld(false);
        this.accelXSignal = pigeon.getAccelerationX(false);
        this.accelYSignal = pigeon.getAccelerationY(false);
        this.accelZSignal = pigeon.getAccelerationZ(false);
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
        return Rotation2d.fromDegrees(rollSignal.refresh().getValueAsDouble());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pitchSignal.refresh().getValueAsDouble());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(yawSignal.refresh().getValueAsDouble());
    }

    @Override
    public Rotation2d getVelocityZ() {
        return Rotation2d.fromDegrees(velZSignal.refresh().getValueAsDouble());
    }

    @Override
    public Rotation2d getVelocityX() {
        return Rotation2d.fromDegrees(velXSignal.refresh().getValueAsDouble());
    }   

    @Override
    public Rotation2d getVelocityY() {
        return Rotation2d.fromDegrees(velYSignal.refresh().getValueAsDouble());
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.inverted = inverted;
        }
    }

    @Override
    public boolean isInverted() {
        return config != null && config.inverted;
    }

    @Override
    public boolean isConnected() {
        return yawSignal.refresh().getStatus().isOK();
    }

    @Override
    public double getAccelX() {
        return accelXSignal.refresh().getValueAsDouble();
    }

    @Override
    public double getAccelY() {
        return accelYSignal.refresh().getValueAsDouble();
    }

    @Override
    public double getAccelZ() {
        return accelZSignal.refresh().getValueAsDouble();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        if (yaw == null) {
            return;
        }
        pigeon.setYaw(yaw.getDegrees());
    }

    @Override
    public void update() {
        BaseStatusSignal.refreshAll(
                yawSignal,
                pitchSignal,
                rollSignal,
                velXSignal,
                velYSignal,
                velZSignal,
                accelXSignal,
                accelYSignal,
                accelZSignal);
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
