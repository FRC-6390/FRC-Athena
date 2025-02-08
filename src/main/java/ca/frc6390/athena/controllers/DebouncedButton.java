package ca.frc6390.athena.controllers;

import ca.frc6390.athena.commands.RunnableTrigger;
import edu.wpi.first.wpilibj.GenericHID;

public class DebouncedButton extends RunnableTrigger {
    private static float DEFUALT_DEBOUNCE_PERIOD = 0.5f;
    private Debouncer debouncer;

    public DebouncedButton(GenericHID joystick, int buttonNumber) {
        this(joystick, buttonNumber,DEFUALT_DEBOUNCE_PERIOD);
    }

    public DebouncedButton(GenericHID joystick, int buttonNumber, float debouncePeriod) {
        super(() -> joystick.getRawButton(buttonNumber));
        debouncer = new Debouncer(this, debouncePeriod);
    }

    public boolean debounced() {
        return debouncer.get();
    }
    
}
