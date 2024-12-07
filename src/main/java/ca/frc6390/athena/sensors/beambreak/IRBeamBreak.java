package ca.frc6390.athena.sensors.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IRBeamBreak {

    private final DigitalInput input;
    private final Trigger trigger;

    /**
     * Constructs an IRBeamBreak sensor.
     *
     * @param port The digital input port number where the beam break sensor is connected.
     */
    public IRBeamBreak(int port) {
        this.input = new DigitalInput(port);
        this.trigger = new Trigger(this::isBroken);
    }

    /**
     * Checks if the IR beam is broken.
     *
     * @return true if the beam is broken, false otherwise.
     */
    public boolean isBroken() {
        return !input.get();
    }

    /**
     * Registers an action to perform when the beam is broken.
     *
     * @param action The action to perform.
     */
    public void onBreak(Command action) {
        trigger.onTrue(action);
    }

    /**
     * Registers an action to perform when the beam is restored.
     *
     * @param action The action to perform.
     */
    public void onRestore(Command action) {
        trigger.onFalse(action);
    }

     /**
     * Registers an action to perform when the beam is broken.
     *
     * @param action The action to perform.
     */
    public void onBreak(Runnable action) {
        trigger.onTrue(new InstantCommand(action));
    }

    /**
     * Registers an action to perform when the beam is restored.
     *
     * @param action The action to perform.
     */
    public void onRestore(Runnable action) {
        trigger.onFalse(new InstantCommand(action));
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
