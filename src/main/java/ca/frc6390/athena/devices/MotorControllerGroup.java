package ca.frc6390.athena.devices;

import java.util.Arrays;
import java.util.List;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class MotorControllerGroup implements RobotSendableDevice{
    
    private MotorController[] controllers;
    private EncoderGroup encoders;

    public MotorControllerGroup(MotorControllerConfig... configs){
       this(List.of(configs).stream().map(MotorController::fromConfig).toArray(MotorController[]::new));
    }

    public MotorControllerGroup(MotorController... controllers){
        this.controllers = controllers;
        this.encoders = EncoderGroup.fromMotorGroup(this);
    }
    
    public MotorControllerGroup(MotorController leftMotor, MotorController rightMotor){
        this(new MotorController[] {leftMotor, rightMotor});
    }

    public static MotorControllerGroup fromConfigs(MotorControllerConfig... configs){
        return new MotorControllerGroup(Arrays.stream(configs).map(MotorController::fromConfig).toArray(MotorController[]::new));
    }

    public MotorController[] getControllers() {
        return controllers;
    }

    public MotorControllerGroup setNeutralMode(MotorNeutralMode mode){
        for (MotorController motorController : controllers) {
            motorController.setNeutralMode(mode);
        }
        return this;
    }

    public MotorControllerGroup setEncoders(EncoderGroup encoders) {
        this.encoders = encoders;
        return this;
    }

    public void setCurrentLimit(double currentLimit){
        for (MotorController motorController : controllers) {
            motorController.setCurrentLimit(currentLimit);
        }
    }

    public void setPosition(double position){
        for (MotorController motorController : controllers) {
            motorController.setPosition(position);
        }
    }

    public void setPID(PIDController pid){
        for (MotorController motorController : controllers) {
            motorController.setPID(pid);
        }
    }

    public void setPID(double p, double i, double d){
        for (MotorController motorController : controllers) {
            motorController.setPID(p,i,d);
        }
    }

    public void setSpeed(double speed){
        for (MotorController motorController : controllers) {
            motorController.setSpeed(speed);
        }
    }

    public void setVoltage(double voltage){
        for (MotorController motorController : controllers) {
            motorController.setVoltage(voltage);
        }
    }

    public void stopMotors(){
        for (MotorController motorController : controllers) {
            motorController.stopMotor();
        }
    }

    public boolean allMotorsConnected(){
        for (MotorController motorController : controllers) {
            if(!motorController.isConnected()){
                return false;
            }
        }
        return true;
    }

    public void update(){
        for (MotorController motorController : controllers) {
            motorController.update();
        }
    }

    public EncoderGroup getEncoderGroup(){
        return encoders;
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        
        for (MotorController motorController : controllers) {
            motorController.shuffleboard(layout.getLayout(motorController.getName(), BuiltInLayouts.kList), level);
        }

        return layout;
    }


}
