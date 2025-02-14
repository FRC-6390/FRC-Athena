package ca.frc6390.athena.devices;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class IMU extends VirtualIMU{

    private final DoubleSupplier getRoll, getPitch, getYaw, getAccelX, getAccelY, getAccelZ;
    private double roll, pitch, yaw, accelX, accelY, accelZ;
    private boolean inverted = false;

    public enum IMUType{
        CTREPigeon2,
        WPILibADXRS450
    }

    public record IMUConfig(IMUType type, int id, String canbus, boolean inverted) {
        public IMUConfig(IMUType type, int id){
            this(type, id, "rio", false);
        }

        public IMUConfig(IMUType type){
            this(type, -1);
        }

        public IMUConfig setCanbus(String canbus){
            return new IMUConfig(type, id, canbus, inverted);
        }

        public IMUConfig setId(int id){
            return new IMUConfig(type, id, canbus, inverted);
        }

        public IMUConfig setInverted(boolean inverted){
            return new IMUConfig(type, id, canbus, inverted);
        }
    }

    public IMU(DoubleSupplier getRoll, DoubleSupplier getPitch, DoubleSupplier getYaw, DoubleSupplier getAccelX, DoubleSupplier getAccelY, DoubleSupplier getAccelZ){
        this.getRoll = getRoll;
        this.getPitch = getPitch;
        this.getYaw = getYaw;
        this.getAccelX = getAccelX;
        this.getAccelY = getAccelY;
        this.getAccelZ = getAccelZ;
        addVirtualAxis("driver", this::getYaw);
    }

    public IMU(Pigeon2 gyro){
        this(
            ()-> gyro.getRoll(true).getValueAsDouble(), 
            ()-> gyro.getPitch(true).getValueAsDouble(), 
            ()-> gyro.getYaw(true).getValueAsDouble(), 
            ()-> gyro.getAccelerationX(true).getValueAsDouble(), 
            ()-> gyro.getAccelerationY(true).getValueAsDouble(), 
            ()-> gyro.getAccelerationZ(true).getValueAsDouble());
    }

    public IMU(ADXRS450_Gyro gyro){
        this(
            ()-> 0d, 
            ()-> 0d, 
            ()-> gyro.getAngle(), 
            ()-> 0d, 
            ()-> 0d, 
            ()-> gyro.getRate());
    }
    
    public void setYaw(double yaw) {
        this.setYaw(Rotation2d.fromDegrees(yaw));
    }

    public void setYaw(Rotation2d yaw) {
        setVirtualAxis("driver", yaw);
    }

    public Rotation2d getRoll(){
        return Rotation2d.fromDegrees(roll);
    }

    public Rotation2d getPitch(){
        return Rotation2d.fromDegrees(pitch);
    }

    public Rotation2d getYaw(){
        return Rotation2d.fromDegrees(yaw);
    }

    public double getAccelerationX(){
        return accelX;
    }

    public double getAccelerationY(){
        return accelY;
    }

    public double getAccelerationZ(){
        return accelZ;
    }

    public void setInverted(boolean inverted){
        this.inverted = inverted;
    }

    public boolean isInverted() {
        return inverted;
    }

    public void update(){
       roll =  inverted ? -getRoll.getAsDouble() : getRoll.getAsDouble();
       pitch = inverted ? -getPitch.getAsDouble() : getPitch.getAsDouble();
       yaw = inverted ? -getYaw.getAsDouble() : getYaw.getAsDouble();
       accelX = inverted ? -getAccelX.getAsDouble() : getAccelX.getAsDouble();
       accelY = inverted ? -getAccelY.getAsDouble() : getAccelY.getAsDouble();
       accelZ = inverted ? -getAccelZ.getAsDouble() : getAccelZ.getAsDouble();
    }

    public IMU applyConfig(IMUConfig config){
        setInverted(config.inverted);
        return this;
    }

    public static IMU createCTREPigeon2(int id){
        return fromConfig(new IMUConfig(IMUType.CTREPigeon2, id));
    }

    public static IMU createCTREPigeon2(int id, String canbus){
        return fromConfig(new IMUConfig(IMUType.CTREPigeon2, id, canbus, false));
    }

    public static IMU fromConfig(IMUConfig config){
        switch (config.type) {
            case CTREPigeon2:
                return new IMU(new Pigeon2(config.id, config.canbus)).applyConfig(config);
            case WPILibADXRS450:
                return new IMU(new ADXRS450_Gyro(Port.kMXP)).applyConfig(config);
            default:
                return null;
        }
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
}
