package ca.frc6390.athena.controllers;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;

public class DelayedOutput implements BooleanSupplier {
    private final BooleanSupplier input;
    private final double delay;
    private double startTime = Double.NaN; // Not started

    public DelayedOutput(BooleanSupplier input, double delay) {
        this.input = input;
        this.delay = delay;
    }

    @Override
    public boolean getAsBoolean() {
        if (input.getAsBoolean()) {
            // Start the timer if it hasn't been started already
            if (Double.isNaN(startTime)) {
                startTime = Timer.getFPGATimestamp();
            }
            // Check if the input has been high for at least 'delay' seconds
            if (Timer.getFPGATimestamp() - startTime >= delay) {
                return true;
            }
        } else {
            // Reset the timer if input goes false
            startTime = Double.NaN;
        }
        return false;
    }
}
