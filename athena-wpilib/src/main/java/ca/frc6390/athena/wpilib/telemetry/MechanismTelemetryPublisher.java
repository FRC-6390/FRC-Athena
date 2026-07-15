package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;

/** Publishes sparse custom telemetry and synchronizes live tunables over NT4. */
public final class MechanismTelemetryPublisher {
    private static final double CUSTOM_PERIOD_SECONDS = 0.10;
    private final NetworkTableInstance instance;
    private final Map<String, NetworkTableEntry> entries = new LinkedHashMap<>();
    private final Set<String> reportedFailures = new HashSet<>();
    private double lastCustomPublish = Double.NEGATIVE_INFINITY;

    public MechanismTelemetryPublisher() { this(NetworkTableInstance.getDefault()); }

    MechanismTelemetryPublisher(NetworkTableInstance instance) {
        this.instance = Objects.requireNonNull(instance, "instance");
    }

    public void publish(Map<String, TelemetryValue> values) {
        publish(values, Timer.getFPGATimestamp());
    }

    void publish(Map<String, TelemetryValue> values, double timestampSeconds) {
        Objects.requireNonNull(values, "values");
        boolean publishCustom = timestampSeconds - lastCustomPublish >= CUSTOM_PERIOD_SECONDS;
        if (publishCustom) lastCustomPublish = timestampSeconds;
        values.forEach((path, value) -> {
            if (!value.writable() && !publishCustom) return;
            String topic = "/Athena/Mechanisms/" + clean(path);
            NetworkTableEntry entry = entries.computeIfAbsent(topic, instance::getEntry);
            try {
                if (value.writable()) sync(entry, value); else write(entry, value);
            } catch (RuntimeException exception) {
                if (reportedFailures.add(path)) {
                    DriverStation.reportWarning(
                            "Athena telemetry '" + path + "' is unavailable: " + exception.getMessage(), false);
                }
            }
        });
    }

    private static void sync(NetworkTableEntry entry, TelemetryValue value) {
        Object current = value.value();
        if (!entry.exists()) {
            setDefault(entry, value.type(), current);
            return;
        }
        switch (value.type()) {
            case NUMBER -> value.set(entry.getDouble(((Number) current).doubleValue()));
            case BOOLEAN -> value.set(entry.getBoolean((Boolean) current));
            case STRING -> value.set(entry.getString(String.valueOf(current)));
        }
    }

    private static void write(NetworkTableEntry entry, TelemetryValue value) {
        Object current = value.value();
        switch (value.type()) {
            case NUMBER -> entry.setDouble(((Number) current).doubleValue());
            case BOOLEAN -> entry.setBoolean((Boolean) current);
            case STRING -> entry.setString(String.valueOf(current));
        }
    }

    private static void setDefault(NetworkTableEntry entry, TelemetryValue.Type type, Object value) {
        switch (type) {
            case NUMBER -> entry.setDefaultDouble(((Number) value).doubleValue());
            case BOOLEAN -> entry.setDefaultBoolean((Boolean) value);
            case STRING -> entry.setDefaultString(String.valueOf(value));
        }
    }

    private static String clean(String path) {
        return path == null ? "value" : path.replaceAll("[^A-Za-z0-9_./-]", "_").replaceAll("^/+", "");
    }
}
