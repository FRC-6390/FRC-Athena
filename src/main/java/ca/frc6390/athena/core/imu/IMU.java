package ca.frc6390.athena.core.imu;

public interface IMU {

    void setYaw(double yaw);

    double getRoll();
    double getPitch();
    double getYaw();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    void update();
}
