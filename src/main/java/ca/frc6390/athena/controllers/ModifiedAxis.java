package ca.frc6390.athena.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;

public class ModifiedAxis implements DoubleSupplier {
    private static final double DEFUALT_DEADBAND = 0.1d;
    private DoubleSupplier input;
    private double deadband;
    private boolean doSquaring = true;
    
    public ModifiedAxis(DoubleSupplier input){
        this(input, DEFUALT_DEADBAND);
    }

    public ModifiedAxis(GenericHID input, int port){
        this(input, port, DEFUALT_DEADBAND);
    }

    public ModifiedAxis(GenericHID input, int port, double deadband){
        this(() -> input.getRawAxis(port), DEFUALT_DEADBAND);
    }

    public ModifiedAxis(DoubleSupplier input, double deadband){
        this.input = input;
        this.deadband = deadband;
    }

    public ModifiedAxis withoutSquaring(){
        doSquaring = false;
        return this;
    }

    private double applyDeadband(double value){
        if (Math.abs(value) <= deadband) return 0.0;
        return value > 0.0 ? (value - deadband) / (1.0 - deadband) : (value + deadband) / (1.0 - deadband);
    }

    private double sqaureAxis(double value){
        return Math.copySign(value*value, value);
    }

    @Override
    public double getAsDouble() {
        double value = applyDeadband(input.getAsDouble());
        return doSquaring ? sqaureAxis(value) : value;
    }
}
