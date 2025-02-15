package ca.frc6390.athena.devices;

import java.util.Arrays;

import ca.frc6390.athena.devices.MotorController.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;

public class MotorControllerGroup {
    
    private MotorController[] controllers;

    public MotorControllerGroup(MotorController... controllers){
        this.controllers = controllers;
    }
    
    public MotorControllerGroup(MotorController leftMotor, MotorController rightMotor){
        this.controllers = new MotorController[] {leftMotor, rightMotor};
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

}
