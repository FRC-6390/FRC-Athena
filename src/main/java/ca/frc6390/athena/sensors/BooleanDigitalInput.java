package ca.frc6390.athena.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class BooleanDigitalInput extends DigitalInput implements BooleanSupplier{

    public BooleanDigitalInput(int channel) {
        super(channel);
    }
    
    @Override
    public boolean getAsBoolean() {
        return super.get();
    }
}
