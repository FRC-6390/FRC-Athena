package ca.frc6390.athena.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ModifiedAxis implements DoubleSupplier {
    private static final double DEFUALT_DEADBAND = 0.01d;
    private DoubleSupplier input;
    private double deadzone;
    private boolean doSquaring = true;
    
    public ModifiedAxis(DoubleSupplier input){
        this(input, DEFUALT_DEADBAND);
    }

    public ModifiedAxis(GenericHID input, int port){
        this(input, port, DEFUALT_DEADBAND);
    }

    public ModifiedAxis(GenericHID input, int port, double deadzone){
        this(() -> input.getRawAxis(port), deadzone);
    }

    public ModifiedAxis(DoubleSupplier input, double deadzone){
        this.input = input;
        this.deadzone = deadzone;
    }

    public ModifiedAxis setDeadzone(double deadzone) {
        this.deadzone = deadzone;
        return this;
    }

    public ModifiedAxis withoutSquaring(){
        doSquaring = false;
        return this;
    }

    private double applyDeadzone(double value){
        if (Math.abs(value) <= deadzone) return 0.0;
        return value > 0.0 ? (value - deadzone) / (1.0 - deadzone) : (value + deadzone) / (1.0 - deadzone);
    }

    public Trigger tiggerAt(double value) {
        return new Trigger(() -> {return getAsDouble() >= value;});
    }

    private double sqaureAxis(double value){
        return Math.copySign(value*value, value);
    }

    @Override
    public double getAsDouble() {
        double value = applyDeadzone(input.getAsDouble());
        return doSquaring ? sqaureAxis(value) : value;
    }
}
