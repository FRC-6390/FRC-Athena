package ca.frc6390.athena.sensors;

import ca.frc6390.athena.commands.RunnableTrigger;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnhancedDigitalInput extends RunnableTrigger{

    private final DigitalInput input;

    public EnhancedDigitalInput(int channel) {
        this(channel, false);
    }

    public EnhancedDigitalInput(int channel, boolean inverted) {
        this(new DigitalInput(channel), inverted);
    }


    public EnhancedDigitalInput(DigitalInput input, boolean inverted) {
        super(() -> inverted ? !input.get() : input.get());
        this.input = input;
    }

    public void close(){
        input.close();
    }
}
