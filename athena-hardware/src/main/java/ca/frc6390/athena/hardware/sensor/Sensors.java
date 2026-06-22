package ca.frc6390.athena.hardware.sensor;

import java.util.function.Consumer;

/**
 * Entry points for sensor wrapper declarations.
 */
public final class Sensors {
    private Sensors() {
    }

    /**
     * Creates and lowers a named sensor.
     *
     * @param ownerPath owner path
     * @param name sensor name
     * @param configure sensor configuration
     * @return sensor spec
     */
    public static SensorSpec sensor(String ownerPath, String name, Consumer<SensorConfig> configure) {
        SensorConfig config = SensorConfig.create();
        if (configure != null) {
            configure.accept(config);
        }
        return config.toSpec(ownerPath, name);
    }
}
