package ca.frc6390.athena.controllers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;

public class Debouncer {
    
    private BooleanSupplier input;
    private double latest = 0;
    private double debounce_period;

    public Debouncer(BooleanSupplier input){
        this.input = input;
        this.debounce_period = .5;
    }
    public Debouncer(BooleanSupplier input, float period){
        this.input = input;
        this.debounce_period = period;
    }

    public boolean get(){
        double now = Timer.getFPGATimestamp();
        if(input.getAsBoolean())
            if((now-latest) > debounce_period){
                latest = now;
                return true;
            }
        return false;
    }
}
