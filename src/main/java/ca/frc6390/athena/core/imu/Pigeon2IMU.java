package ca.frc6390.athena.core.imu;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import ca.frc6390.athena.core.RobotIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;

public class Pigeon2IMU implements RobotIMU {

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> roll, pitch, yaw;
    private final StatusSignal<LinearAcceleration> accelX, accelY, accelZ;
    private Rotation2d fieldYawOffset;

    public Pigeon2IMU(int id, String canbus) {
        this.pigeon = new Pigeon2(id, canbus);
        roll = pigeon.getRoll();
        pitch = pigeon.getPitch();
        yaw = pigeon.getYaw();
        accelX = pigeon.getAccelerationX();
        accelY = pigeon.getAccelerationY();
        accelZ = pigeon.getAccelerationZ();
        fieldYawOffset = new Rotation2d();
    }
    
    public Pigeon2 getPigeon() {
        return pigeon;
    }
   
    @Override
    public void setYaw(double yaw) {
        fieldYawOffset = fieldYawOffset.minus(Rotation2d.fromDegrees(getYaw().getDegrees() - yaw));
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
    public Rotation2d getAccelerationX() {
        return Rotation2d.fromDegrees(accelX.getValueAsDouble());
    }

    @Override
    public Rotation2d getAccelerationY() {
        return Rotation2d.fromDegrees(accelY.getValueAsDouble());
    }

    @Override
    public Rotation2d getAccelerationZ() {
        return Rotation2d.fromDegrees(accelZ.getValueAsDouble());
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


    @Override
    public void setFieldYaw(Rotation2d fieldYaw) {
         fieldYawOffset = getYaw().minus(fieldYaw);
    }

    @Override
    public Rotation2d getFieldYaw() {
       return getYaw().minus(fieldYawOffset);
    }

    @Override
    public Rotation2d getFieldYawOffset() {
       return fieldYawOffset;
    }

    @Override
    public void setFieldYawOffset(Rotation2d yaw) {
        fieldYawOffset = yaw;
    }
}
