package ca.frc6390.athena.core.imu;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMUHIL {

    void setYaw(double yaw);

    Rotation2d getRoll();
    Rotation2d getPitch();
    Rotation2d getYaw();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    void update();
}
