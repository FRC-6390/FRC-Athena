package ca.frc6390.athena.mechanisms;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ProfiledMechanism {
    
    private final MotorController motorController;
    private final Encoder encoder;
    private final ProfiledPIDController controller;

    private RunnableTrigger upperLimit, lowerLimit, homeLimit;
    private double setpoint = 0;

    public ProfiledMechanism(MotorController motorController, ProfiledPIDController controller){
        this.motorController = motorController;
        this.encoder = motorController.getRelativeEncoder();
        this.controller = controller;
    } 

    public ProfiledMechanism(MotorController motorController, Encoder encoder, ProfiledPIDController controller){
        this.motorController = motorController;
        this.encoder = encoder;
        this.controller = controller;
    }   

    public ProfiledMechanism withUpperLimitSwitch(RunnableTrigger upperLimit){
        this.upperLimit = upperLimit;
        return this;
    }

    public ProfiledMechanism withUpperLimitSwitch(RunnableTrigger upperLimit, double maxPosition){
        upperLimit.whileTrue(() -> encoder.setPosition(maxPosition));
        return withUpperLimitSwitch(upperLimit);
    }

    public ProfiledMechanism withLowerLimitSwitch(RunnableTrigger lowerLimit){
        this.lowerLimit = lowerLimit;
        return this;
    }

    public ProfiledMechanism withLowerLimitSwitch(RunnableTrigger lowerLimit, double minPosition){
        lowerLimit.whileTrue(() -> encoder.setPosition(minPosition));
        return withLowerLimitSwitch(lowerLimit);
    }

    public ProfiledMechanism withHomeLimitSwitch(RunnableTrigger homeLimit){
        this.homeLimit = homeLimit;
        return this;
    }

    public ProfiledMechanism withHomeLimitSwitch(RunnableTrigger homeLimit, double homePosition){
        homeLimit.whileTrue(() -> encoder.setPosition(homePosition));
        return withHomeLimitSwitch(homeLimit);
    }

    private boolean getLimitValue(BooleanSupplier limit){
        return limit != null ? limit.getAsBoolean() : false;
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    //will call encoder update loop
    public void update(){
        
        encoder.update();

        double speed = controller.calculate(encoder.getRotations(), setpoint);
        
        if(getLimitValue(lowerLimit) && speed < 0){
            speed = 0;
        }

        if(getLimitValue(upperLimit) && speed > 0){
            speed = 0;
        }

        motorController.setSpeed(speed);
    }


}
