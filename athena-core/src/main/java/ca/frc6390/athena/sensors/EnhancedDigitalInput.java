package ca.frc6390.athena.sensors;

import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import java.util.concurrent.atomic.AtomicBoolean;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class EnhancedDigitalInput extends RunnableTrigger implements RobotSendableDevice{

    private final DigitalInput input;
    private final DIOSim dioSim;
    private final AtomicBoolean inverted;
    private final DelayedOutput delayedOutput;

    public EnhancedDigitalInput(int channel) {
        this(channel, false);
    }

    public EnhancedDigitalInput(int channel, boolean inverted) {
        this(new DigitalInput(channel), inverted);
    }

    public EnhancedDigitalInput(int channel, boolean inverted, double delaySeconds) {
        this(new DigitalInput(channel), inverted, delaySeconds);
    }

    public EnhancedDigitalInput(DigitalInput input, boolean inverted) {
        this(input, inverted, 0);
    }

    public EnhancedDigitalInput(DigitalInput input, boolean inverted, double delaySeconds) {
        this(input, createInversionFlag(inverted), delaySeconds);
    }

    private EnhancedDigitalInput(DigitalInput input, AtomicBoolean inverted, double delaySeconds) {
        this(input, inverted, new DelayedOutput(() -> readRaw(input, inverted), delaySeconds));
    }

    private EnhancedDigitalInput(DigitalInput input, AtomicBoolean inverted,
                                 DelayedOutput delayedOutput) {
        super(delayedOutput);
        this.input = input;
        this.inverted = inverted;
        this.delayedOutput = delayedOutput;
        this.dioSim = RobotBase.isSimulation() ? new DIOSim(input) : null;
    }

    public void close(){
        input.close();
    }

    public void setInverted(boolean inverted) {
        this.inverted.set(inverted);
    }

    public boolean isInverted() {
        return inverted.get();
    }

    public EnhancedDigitalInput setDelay(double delaySeconds) {
        delayedOutput.setDelay(delaySeconds);
        return this;
    }

    public boolean getRawValue() {
        return readRaw(input, inverted);
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
        boolean raw = inverted.get() ? !triggered : triggered;
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

    private static boolean readRaw(DigitalInput input, AtomicBoolean inverted) {
        boolean value = input.get();
        return inverted.get() ? !value : value;
    }

    private static AtomicBoolean createInversionFlag(boolean inverted) {
        return new AtomicBoolean(inverted);
    }
}
