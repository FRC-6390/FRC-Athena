package ca.frc6390.athena.core;

import java.util.Map;

import ca.frc6390.athena.core.imu.Pigeon2IMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public interface RobotIMU {

    void setYaw(double yaw);

    Rotation2d getRoll();
    Rotation2d getPitch();
    Rotation2d getYaw();

    Rotation2d getAccelerationX();
    Rotation2d getAccelerationY();
    Rotation2d getAccelerationZ();

    void update();

    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        layout.addDouble("Yaw",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Roll",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Pitch",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 3);

        layout.addDouble("Acceleration X",() -> getAccelerationX().getDegrees()).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1);
        layout.addDouble("Acceleration Y",() -> getAccelerationY().getDegrees()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 2);
        layout.addDouble("Acceleration Z",() -> getAccelerationZ().getDegrees()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 3);
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
