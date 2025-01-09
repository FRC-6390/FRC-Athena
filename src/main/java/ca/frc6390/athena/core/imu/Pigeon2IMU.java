package ca.frc6390.athena.core.imu;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;

public class Pigeon2IMU implements IMU {

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
    public double getRoll() {
        return roll.getValueAsDouble();
    }

    @Override
    public double getPitch() {
        return pitch.getValueAsDouble();
    }

    @Override
    public double getYaw() {
        return yaw.getValueAsDouble();
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
