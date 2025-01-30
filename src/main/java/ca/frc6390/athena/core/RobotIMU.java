package ca.frc6390.athena.core;

import java.util.Map;

import ca.frc6390.athena.core.imu.Pigeon2IMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public interface RobotIMU {
    //TODO add the ability to have a driver heading and an absolute heading relative to the field
    void setYaw(double yaw);
    void setFieldYawOffset(Rotation2d yaw);
    void setFieldYaw(Rotation2d yaw);
    
    Rotation2d getFieldYawOffset();


    Rotation2d getRoll();
    Rotation2d getPitch();
    Rotation2d getYaw();
    Rotation2d getFieldYaw();


    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    void update();

    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));

        layout.addDouble("Field Yaw Offset",() -> getFieldYawOffset().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Field Yaw",() -> getFieldYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Yaw",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Roll",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Pitch",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 3);

        layout.addDouble("Acceleration X",() -> getAccelerationX()).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1);
        layout.addDouble("Acceleration Y",() -> getAccelerationY()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Acceleration Z",() -> getAccelerationZ()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 3);
        return layout;
    }

    public static Pigeon2IMU createFromPigeon2(int id) {
        return createFromPigeon2(id, "rio");
    }

    public static Pigeon2IMU createFromPigeon2(int id, String canbus) {
        Pigeon2IMU pigeonIMU = new Pigeon2IMU(id, canbus);
        return pigeonIMU;
    }
}
