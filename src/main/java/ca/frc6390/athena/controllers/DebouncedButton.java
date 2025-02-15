package ca.frc6390.athena.controllers;

import java.util.function.BooleanSupplier;

import ca.frc6390.athena.commands.RunnableTrigger;
import edu.wpi.first.wpilibj.GenericHID;

public class DebouncedButton extends RunnableTrigger {
    private static float DEFUALT_DEBOUNCE_PERIOD = 0.5f;
    private Debouncer debouncer;

    
    public DebouncedButton(GenericHID joystick, int buttonNumber) {
        this(() -> joystick.getRawButton(buttonNumber));
    }

    public DebouncedButton(BooleanSupplier button) {
        this(button,DEFUALT_DEBOUNCE_PERIOD);
    }

    public DebouncedButton(BooleanSupplier button, float debouncePeriod) {
        super(button);
        debouncer = new Debouncer(this, debouncePeriod);
    }

    public void setDebouncedPeriod(double period){
        debouncer.setPeriod(period);
    }

    public boolean debounced() {
        return debouncer.getAsBoolean();
    }
    
}
