package ca.frc6390.athena.core;

import java.util.Map;

import ca.frc6390.athena.core.imu.IMUHIL;
import ca.frc6390.athena.core.imu.VirtualIMU;
import ca.frc6390.athena.core.imu.devices.Pigeon2IMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class RobotIMU<T extends IMUHIL> extends VirtualIMU implements IMUHIL {

    private final T imu;

    public RobotIMU(T imu){
        this.imu = imu;
        addVirtualAxis("driver", this::getYaw);
    }

    public void setYaw(double yaw) {
        // recalculateVirtualOffsets(Rotation2d.fromDegrees(yaw));
        // imu.setYaw(yaw);
        this.setYaw(Rotation2d.fromDegrees(yaw));
    }

    public void setYaw(Rotation2d yaw) {
        setVirtualAxis("driver", yaw);
    }

    public Rotation2d getRoll(){
        return imu.getRoll();
    }
    
    public Rotation2d getPitch(){
        return imu.getPitch();
    }
    public Rotation2d getYaw(){
        return imu.getYaw();
    }

    public double getRollAsDegrees(){
        return imu.getRoll().getDegrees();
    }
    public double getPitchAsDegrees(){
        return imu.getPitch().getDegrees();
    }
    public double getYawAsDegrees(){
        return imu.getYaw().getDegrees();
    }

    public double getRollAsRadians(){
        return imu.getRoll().getRadians();
    }
    public double getPitchAsRadians(){
        return imu.getPitch().getRadians();
    }
    public double getYawAsRadians(){
        return imu.getYaw().getRadians();
    }

    public double getAccelerationX(){
        return imu.getAccelerationX();
    }
    public double getAccelerationY(){
        return imu.getAccelerationY();
    }
    public double getAccelerationZ(){
        return imu.getAccelerationZ();
    }

    public void update(){
        imu.update();
    }

    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));

        layout.addDouble("Raw Yaw",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Yaw",() -> getVirtualAxis("driver").getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Roll",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Pitch",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 3);

        layout.addDouble("Acceleration X",() -> getAccelerationX()).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1);
        layout.addDouble("Acceleration Y",() -> getAccelerationY()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Acceleration Z",() -> getAccelerationZ()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 3);
        return layout;
    }

    public static RobotIMU<Pigeon2IMU> createFromPigeon2(int id) {
        return createFromPigeon2(id, "rio");
    }

    public static RobotIMU<Pigeon2IMU> createFromPigeon2(int id, String canbus) {
        return new RobotIMU<Pigeon2IMU>(new Pigeon2IMU(id, canbus));
    }
}
