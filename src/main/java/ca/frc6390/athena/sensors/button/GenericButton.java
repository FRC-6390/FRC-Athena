package ca.frc6390.athena.sensors.button;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GenericButton {

    private final DigitalInput input;
    private final Trigger trigger;

    /**
     * Constructs an IRBeamBreak sensor.
     *
     * @param port The digital input port number where the beam break sensor is connected.
     */
    public GenericButton(int port) {
        this.input = new DigitalInput(port);
        this.trigger = new Trigger(this::isPressed);
    }

    /**
     * Checks if the IR beam is broken.
     *
     * @return true if the beam is broken, false otherwise.
     */
    public boolean isPressed() {
        return !input.get();
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
     * Registers an action to perform when the button is released.
     *
     * @param action The action to perform.
     */
    public void onRelease(Command action) {
        trigger.onFalse(action);
    }

    /**
     * Registers an action to perform when the button is released.
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
    public Trigger getTrigger() {
        return trigger;
    }

    /**
     * Frees resources associated with the DigitalInput.
     * Call this method when the sensor is no longer needed.
     */
    public void close() {
        input.close();
    }
}
