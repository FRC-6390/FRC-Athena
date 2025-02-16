package ca.frc6390.athena.sensors.button;

import ca.frc6390.athena.sensors.EnhancedDigitalInput;
public class GenericButton extends EnhancedDigitalInput {


    /**
     * Constructs an GenericButton sensor.
     *
     * @param port The digital input port number where the button sensor is connected.
     */
    public GenericButton(int port) {
        this(port, true);
    }
 
    public GenericButton(int port, boolean inverted) {
        super(port, inverted);
    }
}
