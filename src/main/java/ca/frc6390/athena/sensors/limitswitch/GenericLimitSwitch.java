package ca.frc6390.athena.sensors.limitswitch;

import ca.frc6390.athena.commands.RunnableTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class GenericLimitSwitch {

    public record GenericLimitSwitchConfig(int id, boolean inverted, double position, boolean isHardstop) {
        public static GenericLimitSwitchConfig inverted(int id){
            return new GenericLimitSwitchConfig(id ,true, 0, false);
        }

        public static GenericLimitSwitchConfig normal(int id){
            return new GenericLimitSwitchConfig(id ,false, 0, false);
        }

        public GenericLimitSwitchConfig setPosition(double position){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop);
        }

        public GenericLimitSwitchConfig setHardstop(boolean isHardstop){
            return new GenericLimitSwitchConfig(id, inverted, position, isHardstop);
        }

        public GenericLimitSwitch create(){
            return new GenericLimitSwitch(id, inverted).applyConfig(this);
        }

    }

    private final DigitalInput input;
    private final RunnableTrigger trigger;
    private final boolean inverted;
    private double position;
    private boolean isHardstop;

    
    /**
     * Constructs an GenericLimitSwitch sensor.
     *
     * @param port The digital input port number where the switch sensor is connected.
     */
    public GenericLimitSwitch(int port) {
        this(port, true);
    }
 
    public GenericLimitSwitch(int port, boolean inverted) {
        this.input = new DigitalInput(port);
        this.trigger = new RunnableTrigger(this::isPressed);
        this.inverted = inverted;
        this.position = 0;
    }

    public static GenericLimitSwitch fromConfig(GenericLimitSwitchConfig config){
        return new GenericLimitSwitch(config.id, config.inverted);
    }

    public GenericLimitSwitch applyConfig(GenericLimitSwitchConfig config){
        setPosition(config.position);
        setHardstop(config.isHardstop);
        return this;
    }

    /**
     * Checks if the switch is press.
     *
     * @return true if the switch is press, false otherwise.
     */
    public boolean isPressed() {
        return inverted ? !input.get() : input.get();
    }

    /**
     * Registers an action to perform when the buton is pressed.
     *
     * @param action The action to perform.
     */
    public void onPress(Command action) {
        trigger.onTrue(action);
    }

     /**
     * Registers an action to perform when the buton is pressed.
     *
     * @param action The action to perform.
     */
    public void onPress(Runnable action) {
        trigger.onTrue(new InstantCommand(action));
    }

    /**
     * Registers an action to perform when the switch is released.
     *
     * @param action The action to perform.
     */
    public void onRelease(Command action) {
        trigger.onFalse(action);
    }

    /**
     * Registers an action to perform when the switch is released.
     *
     * @param action The action to perform.
     */
    public void onRelease(Runnable action) {
        trigger.onTrue(new InstantCommand(action));
    }

    /**
     * Returns the underlying Trigger object.
     *
     * @return The Trigger instance.
     */
    public RunnableTrigger getTrigger() {
        return trigger;
    }

    /**
     * Frees resources associated with the DigitalInput.
     * Call this method when the sensor is no longer needed.
     */
    public void close() {
        input.close();
    }

    public GenericLimitSwitch setPosition(double position) {
        this.position = position;
        return this;
    }

    public double getPosition() {
        return position;
    }

    public boolean isHardstop() {
        return isHardstop;
    }

    public boolean isInverted() {
        return inverted;
    }
    
    public void setHardstop(boolean isHardstop) {
        this.isHardstop = isHardstop;
    }
    
    
}
