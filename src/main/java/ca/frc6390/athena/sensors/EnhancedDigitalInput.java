package ca.frc6390.athena.sensors;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class EnhancedDigitalInput extends RunnableTrigger implements RobotSendableDevice{

    private final DigitalInput input;
    private boolean inverted;

    public EnhancedDigitalInput(int channel) {
        this(channel, false);
    }

    public EnhancedDigitalInput(int channel, boolean inverted) {
        this(new DigitalInput(channel), inverted);
    }


    public EnhancedDigitalInput(DigitalInput input, boolean inverted) {
        super(() -> inverted ? !input.get() : input.get());
        this.inverted = inverted;
        this.input = input;
    }

    public void close(){
        input.close();
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public boolean isInverted() {
        return inverted;
    }

    public int getPort(){
        return input.getChannel();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        layout.addBoolean("Inverted", this::isInverted);
        layout.addDouble("Port", this::getPort);
        layout.addBoolean("Value", this::getAsBoolean);
        return layout;
    }
}
