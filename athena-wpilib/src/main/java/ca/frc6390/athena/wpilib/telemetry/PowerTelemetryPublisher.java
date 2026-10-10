package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.hardware.power.PowerSnapshot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/** Publishes the electrical and thermal state of the power pass under {@code /Athena/Power}. */
public final class PowerTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/Power";

    private final NetworkTableInstance instance;
    private final Map<String, NetworkTableEntry> entries = new HashMap<>();
    private final Map<String, Map<String, NetworkTableEntry>> motorEntries = new HashMap<>();

    public PowerTelemetryPublisher() {
        this(NetworkTableInstance.getDefault());
    }

    PowerTelemetryPublisher(NetworkTableInstance instance) {
        this.instance = Objects.requireNonNull(instance, "instance");
    }

    /**
     * Publishes one snapshot.
     *
     * @param snapshot electrical state from the latest runtime cycle
     */
    public void publish(PowerSnapshot snapshot) {
        Objects.requireNonNull(snapshot, "snapshot");
        value("Bus/Volts", snapshot.busVolts());
        value("Bus/OpenCircuitVolts", snapshot.openCircuitVolts());
        value("Bus/BatteryOhms", snapshot.batteryOhms());
        value("Bus/FloorVolts", snapshot.floorVolts());
        value("Bus/PredictedVolts", snapshot.predictedVolts());
        value("Bus/BudgetAmps", snapshot.budgetAmps());
        value("Bus/DemandAmps", snapshot.demandAmps());
        value("Bus/Shedding", snapshot.shedding() ? 1.0 : 0.0);
        for (PowerSnapshot.Motor motor : snapshot.motors()) {
            Map<String, NetworkTableEntry> fields = motorEntries.computeIfAbsent(motor.name(), name -> new HashMap<>());
            field(fields, motor.name(), "Demand", motor.demandAmps());
            field(fields, motor.name(), "CapAmps", motor.capAmps());
            field(fields, motor.name(), "Derate", motor.derateFraction());
            field(fields, motor.name(), "TemperatureCelsius", motor.temperatureCelsius());
            field(fields, motor.name(), "Share", motor.shareWeight());
            field(fields, motor.name(), "SustainedAmps", motor.sustainedAmps());
            fields.computeIfAbsent("Status", key -> instance.getEntry(ROOT + "/Motors/" + motor.name() + "/Status"))
                    .setString(motor.status());
        }
    }

    private void value(String path, double value) {
        entries.computeIfAbsent(path, key -> instance.getEntry(ROOT + "/" + key)).setDouble(value);
    }

    private void field(Map<String, NetworkTableEntry> fields, String motor, String name, double value) {
        fields.computeIfAbsent(name, key -> instance.getEntry(ROOT + "/Motors/" + motor + "/" + key))
                .setDouble(value);
    }

    @Override
    public void close() {
        entries.clear();
        motorEntries.clear();
    }
}
