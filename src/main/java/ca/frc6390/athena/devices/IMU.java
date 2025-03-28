package ca.frc6390.athena.devices;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class IMU extends VirtualIMU implements RobotSendableDevice{

    private final DoubleSupplier getRoll, getPitch, getYaw, getVelX, getVelY, getVelZ;
    private double roll, pitch, yaw, velX, velY, velZ;
    private boolean inverted = false;

    public enum IMUType{
        CTREPigeon2,
        WPILibADXRS450,
        NAVX_IC2,
        NAVX_MXP_SPI,
        NAVX_MXP_UART,
        NAVX_USB1,
        NAVX_USB2,

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

    public IMU(DoubleSupplier getRoll, DoubleSupplier getPitch, DoubleSupplier getYaw, DoubleSupplier getvelX, DoubleSupplier getvelY, DoubleSupplier getvelZ){
        this.getRoll = getRoll;
        this.getPitch = getPitch;
        this.getYaw = getYaw;
        this.getVelX = getvelX;
        this.getVelY = getvelY;
        this.getVelZ = getvelZ;
        addVirtualAxis("driver", this::getYaw);
    }

    public IMU(Pigeon2 gyro){
        this(
            ()-> gyro.getRoll(true).getValueAsDouble(), 
            ()-> gyro.getPitch(true).getValueAsDouble(), 
            ()-> gyro.getYaw(true).getValueAsDouble(), 
            ()-> gyro.getAngularVelocityXWorld(true).getValueAsDouble(), 
            ()-> gyro.getAngularVelocityYWorld(true).getValueAsDouble(), 
            ()-> gyro.getAngularVelocityZWorld(true).getValueAsDouble());
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

    public IMU(AHRS gyro){
        this(
            ()-> gyro.getRoll(), 
            ()-> gyro.getPitch(), 
            ()-> gyro.getYaw(), 
            ()-> gyro.getVelocityX(), 
            ()-> gyro.getVelocityY(), 
            ()-> gyro.getVelocityZ());
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

    public Rotation2d getVelocityX(){
        return Rotation2d.fromDegrees(velX);
    }

    public Rotation2d getVelocityY(){
        return Rotation2d.fromDegrees(velY);
    }

    public Rotation2d getVelocityZ(){
        return Rotation2d.fromDegrees(velZ);
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
       velX = inverted ? -getVelX.getAsDouble() : getVelX.getAsDouble();
       velY = inverted ? -getVelY.getAsDouble() : getVelY.getAsDouble();
       velZ = inverted ? -getVelZ.getAsDouble() : getVelZ.getAsDouble();
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
            case NAVX_IC2:
                return new IMU(new AHRS(NavXComType.kI2C)).applyConfig(config);
            case NAVX_MXP_SPI:
                return new IMU(new AHRS(NavXComType.kMXP_SPI)).applyConfig(config);
            case NAVX_MXP_UART:
                return new IMU(new AHRS(NavXComType.kMXP_UART)).applyConfig(config);
            case NAVX_USB1:
                return new IMU(new AHRS(NavXComType.kUSB1)).applyConfig(config);
            case NAVX_USB2:
                return new IMU(new AHRS(NavXComType.kUSB2)).applyConfig(config);
            default:
                return null;
        }
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        layout.withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        layout.addDouble("Yaw",() -> getVirtualAxis("driver").getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
        layout.addDouble("Raw Yaw",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 1);
       
        if(level.equals(SendableLevel.DEBUG)){
            layout.addDouble("Roll",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 2);
            layout.addDouble("Pitch",() -> getYaw().getDegrees()).withWidget(BuiltInWidgets.kGyro).withSize(1, 1).withPosition(0, 3);
    
            layout.addDouble("Velocity X",() -> getVelocityX().getDegrees()).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1);
            layout.addDouble("Velocity Y",() -> getVelocityY().getDegrees()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 2);
            layout.addDouble("Velocity Z",() -> getVelocityZ().getDegrees()).withWidget(BuiltInWidgets.kNumberSlider).withSize(1, 1).withPosition(0, 3);    
        }
        return layout;
    }
}
