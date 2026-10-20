package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.hardware.power.PowerSnapshot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Publishes the electrical and thermal state of the power pass under {@code /Athena/Power}.
 *
 * <p>Entries are resolved once per path. A value is written only when it differs from the last value this publisher
 * wrote to the entry, so an unchanged value costs no NetworkTables call; a changed value is written in the same
 * publish.</p>
 */
public final class PowerTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/Power";
    private static final String[] BUS_FIELDS = {
        "Bus/Volts", "Bus/OpenCircuitVolts", "Bus/BatteryOhms", "Bus/FloorVolts",
        "Bus/PredictedVolts", "Bus/BudgetAmps", "Bus/DemandAmps", "Bus/Shedding"
    };

    private final NetworkTableInstance instance;
    private final NumberChannel[] bus = new NumberChannel[BUS_FIELDS.length];
    private final Map<String, MotorChannels> motorEntries = new HashMap<>();
    /**
     * Channels resolved for each position of the snapshot's motor list, with the name they were resolved for. The
     * power pass keeps its motor order and name instances from cycle to cycle, so the map is only consulted when a
     * position changes.
     */
    private MotorChannels[] motorSlots = new MotorChannels[0];
    private String[] motorSlotNames = new String[0];

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
        busField(0).set(snapshot.busVolts());
        busField(1).set(snapshot.openCircuitVolts());
        busField(2).set(snapshot.batteryOhms());
        busField(3).set(snapshot.floorVolts());
        busField(4).set(snapshot.predictedVolts());
        busField(5).set(snapshot.budgetAmps());
        busField(6).set(snapshot.demandAmps());
        busField(7).set(snapshot.shedding() ? 1.0 : 0.0);
        List<PowerSnapshot.Motor> motors = snapshot.motors();
        int count = motors.size();
        if (motorSlots.length != count) {
            motorSlots = new MotorChannels[count];
            motorSlotNames = new String[count];
        }
        for (int index = 0; index < count; index++) {
            PowerSnapshot.Motor motor = motors.get(index);
            String name = motor.name();
            MotorChannels channels = motorSlots[index];
            if (channels == null || motorSlotNames[index] != name) {
                channels = motorEntries.get(name);
                if (channels == null) {
                    channels = new MotorChannels(instance, name);
                    motorEntries.put(name, channels);
                }
                motorSlots[index] = channels;
                motorSlotNames[index] = name;
            }
            channels.publish(motor);
        }
    }

    private NumberChannel busField(int index) {
        NumberChannel channel = bus[index];
        if (channel == null) {
            channel = new NumberChannel(instance.getEntry(ROOT + "/" + BUS_FIELDS[index]));
            bus[index] = channel;
        }
        return channel;
    }

    @Override
    public void close() {
        java.util.Arrays.fill(bus, null);
        motorEntries.clear();
        motorSlots = new MotorChannels[0];
        motorSlotNames = new String[0];
    }

    /** One numeric entry and the last value written to it. */
    private static final class NumberChannel {
        private final NetworkTableEntry entry;
        private boolean published;
        private long lastBits;

        private NumberChannel(NetworkTableEntry entry) {
            this.entry = entry;
        }

        private void set(double value) {
            long bits = Double.doubleToLongBits(value);
            if (published && bits == lastBits) return;
            entry.setDouble(value);
            lastBits = bits;
            published = true;
        }
    }

    /** One motor's entries under {@code Motors/<name>}. */
    private static final class MotorChannels {
        private final NumberChannel demand;
        private final NumberChannel capAmps;
        private final NumberChannel derate;
        private final NumberChannel temperature;
        private final NumberChannel share;
        private final NumberChannel sustained;
        private final NetworkTableEntry status;
        private String lastStatus;

        private MotorChannels(NetworkTableInstance instance, String motor) {
            String prefix = ROOT + "/Motors/" + motor + "/";
            demand = new NumberChannel(instance.getEntry(prefix + "Demand"));
            capAmps = new NumberChannel(instance.getEntry(prefix + "CapAmps"));
            derate = new NumberChannel(instance.getEntry(prefix + "Derate"));
            temperature = new NumberChannel(instance.getEntry(prefix + "TemperatureCelsius"));
            share = new NumberChannel(instance.getEntry(prefix + "Share"));
            sustained = new NumberChannel(instance.getEntry(prefix + "SustainedAmps"));
            status = instance.getEntry(prefix + "Status");
        }

        private void publish(PowerSnapshot.Motor motor) {
            demand.set(motor.demandAmps());
            capAmps.set(motor.capAmps());
            derate.set(motor.derateFraction());
            temperature.set(motor.temperatureCelsius());
            share.set(motor.shareWeight());
            sustained.set(motor.sustainedAmps());
            String current = motor.status();
            if (lastStatus == null || !lastStatus.equals(current)) {
                status.setString(current);
                lastStatus = current;
            }
        }
    }
}
