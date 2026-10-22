package ca.frc6390.athena.wpilib.telemetry;

import static edu.wpi.first.units.Units.Seconds;

import ca.frc6390.athena.hardware.power.PowerSnapshot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * Publishes the electrical and thermal state of the power pass under {@code /Athena/Power}.
 * {@code Bus/DemandAmps} and {@code Bus/BudgetAmps} use battery-side supply amps; regeneration is
 * excluded from demand. Per-motor {@code Demand}, {@code CapAmps} and {@code SustainedAmps} use
 * winding (stator) amps, estimated when a motor lacks telemetry or an electrical model. A motor cap
 * of infinity is absent, while zero is an active cap. Predictions use the latest hardware samples.
 *
 * <p>Entries are resolved once per path. A value is written only when it differs from the last value this publisher
 * wrote to the entry, so an unchanged value costs no NetworkTables call. Optional count and wall-time budgets spread
 * initial entry creation across publishes, and a diagnostic period can reduce snapshot sampling without changing the
 * power control pass. With the defaults, all values are published in the same call.</p>
 */
public final class PowerTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/Power";
    private static final String[] BUS_FIELDS = {
        "Bus/Volts", "Bus/OpenCircuitVolts", "Bus/BatteryOhms", "Bus/FloorVolts",
        "Bus/PredictedVolts", "Bus/BudgetAmps", "Bus/DemandAmps", "Bus/Shedding"
    };

    private final NetworkTableInstance instance;
    private final LongSupplier nanoTime;
    private final NumberChannel[] bus = new NumberChannel[BUS_FIELDS.length];
    private final Map<String, MotorChannels> motorEntries = new HashMap<>();
    /**
     * Channels resolved for each position of the snapshot's motor list, with the name they were resolved for. The
     * power pass keeps its motor order and name instances from cycle to cycle, so the map is only consulted when a
     * position changes.
     */
    private MotorChannels[] motorSlots = new MotorChannels[0];
    private String[] motorSlotNames = new String[0];
    private int bindBudget = Integer.MAX_VALUE;
    private long bindTimeBudgetNanos = Long.MAX_VALUE;
    private double readOnlyPeriodSeconds;
    private double lastPublishSeconds = Double.NEGATIVE_INFINITY;

    public PowerTelemetryPublisher() {
        this(NetworkTableInstance.getDefault());
    }

    PowerTelemetryPublisher(NetworkTableInstance instance) {
        this(instance, System::nanoTime);
    }

    PowerTelemetryPublisher(NetworkTableInstance instance, LongSupplier nanoTime) {
        this.instance = Objects.requireNonNull(instance, "instance");
        this.nanoTime = Objects.requireNonNull(nanoTime, "nanoTime");
    }

    /** Limits how many new motor channel sets one publish creates; existing channels continue updating. */
    public PowerTelemetryPublisher bindBudget(int motorsPerPublish) {
        if (motorsPerPublish < 1) throw new IllegalArgumentException("Power telemetry bind budget must be positive.");
        bindBudget = motorsPerPublish;
        return this;
    }

    /**
     * Limits wall time spent creating channel sets in a publish. The bus entries or one motor's entries always make
     * progress; an individual NetworkTables call cannot be interrupted. The default has no time limit.
     */
    public PowerTelemetryPublisher bindTimeBudget(Time budget) {
        double seconds = Objects.requireNonNull(budget, "budget").in(Seconds);
        if (!Double.isFinite(seconds) || seconds <= 0.0) {
            throw new IllegalArgumentException("Power telemetry binding time budget must be finite and positive.");
        }
        bindTimeBudgetNanos = Math.max(1L, Math.round(seconds * 1.0e9));
        return this;
    }

    /** Sets the diagnostic publish period. The default publishes each call; this never changes power control. */
    public PowerTelemetryPublisher readOnlyPeriod(Time period) {
        double seconds = Objects.requireNonNull(period, "period").in(Seconds);
        if (!Double.isFinite(seconds) || seconds < 0.0 || seconds > 5.0) {
            throw new IllegalArgumentException("Power telemetry period must be between 0 and 5 seconds.");
        }
        readOnlyPeriodSeconds = seconds;
        return this;
    }

    /**
     * Publishes one snapshot.
     *
     * @param snapshot electrical state from the latest runtime cycle
     */
    public void publish(PowerSnapshot snapshot) {
        publish(snapshot, Timer.getFPGATimestamp());
    }

    /** Samples the power snapshot only when diagnostics are due, avoiding snapshot allocation between publishes. */
    public void publish(Supplier<PowerSnapshot> snapshot) {
        publish(snapshot, Timer.getFPGATimestamp());
    }

    void publish(Supplier<PowerSnapshot> snapshot, double timestampSeconds) {
        Objects.requireNonNull(snapshot, "snapshot");
        if (isDue(timestampSeconds)) publish(snapshot.get(), timestampSeconds);
    }

    private boolean isDue(double timestampSeconds) {
        return timestampSeconds < lastPublishSeconds
                || timestampSeconds - lastPublishSeconds >= readOnlyPeriodSeconds;
    }

    void publish(PowerSnapshot snapshot, double timestampSeconds) {
        Objects.requireNonNull(snapshot, "snapshot");
        if (!isDue(timestampSeconds)) return;
        lastPublishSeconds = timestampSeconds;
        long started = bindTimeBudgetNanos == Long.MAX_VALUE ? 0L : nanoTime.getAsLong();
        boolean boundBus = bus[0] == null;
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
        int bound = 0;
        for (int index = 0; index < count; index++) {
            PowerSnapshot.Motor motor = motors.get(index);
            String name = motor.name();
            MotorChannels channels = motorSlots[index];
            if (channels == null || motorSlotNames[index] != name) {
                channels = motorEntries.get(name);
                if (channels == null) {
                    if (bound >= bindBudget || (boundBus || bound > 0) && bindTimeBudgetNanos != Long.MAX_VALUE
                            && nanoTime.getAsLong() - started >= bindTimeBudgetNanos) continue;
                    channels = new MotorChannels(instance, name);
                    motorEntries.put(name, channels);
                    bound++;
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
        lastPublishSeconds = Double.NEGATIVE_INFINITY;
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
