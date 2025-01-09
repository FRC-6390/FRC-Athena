package ca.frc6390.athena.core;

import ca.frc6390.athena.core.imu.IMU;
import ca.frc6390.athena.core.imu.Pigeon2IMU;

public class RobotIMU<T extends IMU> {

    private final T imu;

    public RobotIMU(T imu) {
        this.imu = imu;
    }

    public static RobotIMU<Pigeon2IMU> createFromPigeon2(int id) {
        return createFromPigeon2(id, "rio");
    }

    public static RobotIMU<Pigeon2IMU> createFromPigeon2(int id, String canbus) {
        Pigeon2IMU pigeonIMU = new Pigeon2IMU(id, canbus);
        return new RobotIMU<>(pigeonIMU);
    }

    public void setYaw(double yaw) {
        imu.setYaw(yaw);
    }

    public double getRoll() {
        return imu.getRoll();
    }

    public double getPitch() {
        return imu.getPitch();
    }

    public double getYaw() {
        return imu.getYaw();
    }

    public double getAccelerationX() {
        return imu.getAccelerationX();
    }

    public double getAccelerationY() {
        return imu.getAccelerationY();
    }

    public double getAccelerationZ() {
        return imu.getAccelerationZ();
    }

    public T getIMU() {
        return imu;
    }

    public void update() {
        imu.update();
    }
}
