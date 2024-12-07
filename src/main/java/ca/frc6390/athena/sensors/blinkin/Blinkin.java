package ca.frc6390.athena.sensors.blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

    private final Spark driver;
    
    public Blinkin(int port) {
        driver = new Spark(port);
    }

    public void set(double value) {
        driver.set(value);
    }
}
