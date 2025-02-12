package ca.frc6390.athena.devices;

import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;

public class MotorControllerGroup {
    
    private MotorController[] controllers;

    public MotorControllerGroup(MotorController... controllers){
        this.controllers = controllers;
    }
    
    public MotorControllerGroup(MotorController leftMotor, MotorController rightMotor){
        this.controllers = new MotorController[] {leftMotor, rightMotor};
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
