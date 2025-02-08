package ca.frc6390.athena.core.imu.devices;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import ca.frc6390.athena.core.imu.IMUHIL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;

public class Pigeon2IMU implements IMUHIL {

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> roll, pitch, yaw;
    private final StatusSignal<LinearAcceleration> accelX, accelY, accelZ;

    public Pigeon2IMU(int id, String canbus) {
        this.pigeon = new Pigeon2(id, canbus);
        roll = pigeon.getRoll();
        pitch = pigeon.getPitch();
        yaw = pigeon.getYaw();
        accelX = pigeon.getAccelerationX();
        accelY = pigeon.getAccelerationY();
        accelZ = pigeon.getAccelerationZ();
    }
    
    public Pigeon2 getPigeon() {
        return pigeon;
    }

    @Override
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(roll.getValueAsDouble());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pitch.getValueAsDouble());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }

    @Override
    public double getAccelerationX() {
        return accelX.getValueAsDouble();
    }

    @Override
    public double getAccelerationY() {
        return accelY.getValueAsDouble();
    }

    @Override
    public double getAccelerationZ() {
        return accelZ.getValueAsDouble();
    }

    @Override
    public void update() {
        roll.refresh();
        pitch.refresh();
        yaw.refresh();
        accelX.refresh();
        accelY.refresh();
        accelZ.refresh();
    }
}
