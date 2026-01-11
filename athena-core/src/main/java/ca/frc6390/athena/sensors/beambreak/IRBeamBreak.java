package ca.frc6390.athena.sensors.beambreak;

import ca.frc6390.athena.sensors.EnhancedDigitalInput;
public class IRBeamBreak extends EnhancedDigitalInput{

 
    /**
     * Constructs an IRBeamBreak sensor.
     *
     * @param port The digital input port number where the beam break sensor is connected.
     */
    public IRBeamBreak(int port) {
       this(port, true);
    }

    public IRBeamBreak(int port, boolean inverted) {
       super(port, inverted);
    }
}
