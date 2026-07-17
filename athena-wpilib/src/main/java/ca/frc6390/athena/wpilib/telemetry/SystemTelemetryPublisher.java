package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.wpilib.system.SystemStatus;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Publishes roboRIO/JVM health and tuning results under {@code /Athena/System}. */
public final class SystemTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/System";
    private final List<NetworkTableEntry> entries = new ArrayList<>();
    private final NetworkTableEntry profile;
    private final NetworkTableEntry target;
    private final NetworkTableEntry pressure;
    private final NetworkTableEntry totalMemory;
    private final NetworkTableEntry availableMemory;
    private final NetworkTableEntry processResident;
    private final NetworkTableEntry heapUsed;
    private final NetworkTableEntry heapMaximum;
    private final NetworkTableEntry directBuffers;
    private final NetworkTableEntry swapTotal;
    private final NetworkTableEntry swapUsed;
    private final NetworkTableEntry tuningComplete;
    private final NetworkTableEntry appliedChanges;
    private final NetworkTableEntry failures;
    private SystemStatus lastStatus;

    public SystemTelemetryPublisher() {
        this(NetworkTableInstance.getDefault());
    }

    SystemTelemetryPublisher(NetworkTableInstance instance) {
        Objects.requireNonNull(instance, "instance");
        profile = entry(instance, "Profile");
        target = entry(instance, "Target");
        pressure = entry(instance, "Pressure");
        totalMemory = entry(instance, "Memory/TotalBytes");
        availableMemory = entry(instance, "Memory/AvailableBytes");
        processResident = entry(instance, "Memory/ProcessResidentBytes");
        heapUsed = entry(instance, "Memory/HeapUsedBytes");
        heapMaximum = entry(instance, "Memory/HeapMaximumBytes");
        directBuffers = entry(instance, "Memory/DirectBufferBytes");
        swapTotal = entry(instance, "Swap/TotalBytes");
        swapUsed = entry(instance, "Swap/UsedBytes");
        tuningComplete = entry(instance, "Tuning/Complete");
        appliedChanges = entry(instance, "Tuning/AppliedChanges");
        failures = entry(instance, "Tuning/Failures");
    }

    public void publish(SystemStatus status) {
        if (status == lastStatus) return;
        lastStatus = status;
        profile.setString(status.profile().name());
        target.setString(status.target());
        pressure.setString(status.pressure().name());
        totalMemory.setInteger(status.totalMemoryBytes());
        availableMemory.setInteger(status.availableMemoryBytes());
        processResident.setInteger(status.processResidentBytes());
        heapUsed.setInteger(status.heapUsedBytes());
        heapMaximum.setInteger(status.heapMaximumBytes());
        directBuffers.setInteger(status.directBufferBytes());
        swapTotal.setInteger(status.swapTotalBytes());
        swapUsed.setInteger(status.swapUsedBytes());
        tuningComplete.setBoolean(status.tuningComplete());
        appliedChanges.setStringArray(status.appliedChanges().toArray(String[]::new));
        failures.setStringArray(status.failures().toArray(String[]::new));
    }

    private NetworkTableEntry entry(NetworkTableInstance instance, String path) {
        NetworkTableEntry entry = instance.getEntry(ROOT + "/" + path);
        entries.add(entry);
        return entry;
    }

    @Override
    public void close() {
        lastStatus = null;
        entries.forEach(NetworkTableEntry::close);
        entries.clear();
    }
}
