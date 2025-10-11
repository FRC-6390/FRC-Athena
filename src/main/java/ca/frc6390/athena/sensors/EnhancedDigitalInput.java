package ca.frc6390.athena.sensors;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class EnhancedDigitalInput extends RunnableTrigger implements RobotSendableDevice{

    private final DigitalInput input;
    private final DIOSim dioSim;
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
        this.dioSim = RobotBase.isSimulation() ? new DIOSim(input) : null;
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

    public boolean supportsSimulation() {
        return dioSim != null;
    }

    public void setSimulatedRawValue(boolean value) {
        if (dioSim == null) {
            return;
        }
        dioSim.setValue(value);
    }

    public void setSimulatedTriggered(boolean triggered) {
        if (dioSim == null) {
            return;
        }
        boolean raw = inverted ? !triggered : triggered;
        dioSim.setValue(raw);
    }

    public int getPort(){
        return input.getChannel();
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        if(level.equals(SendableLevel.DEBUG)){ 
            layout.addBoolean("Inverted", this::isInverted);
            layout.addDouble("Port", this::getPort);
        }
        layout.addBoolean("Value", this::getAsBoolean);
        return layout;
    }
}
