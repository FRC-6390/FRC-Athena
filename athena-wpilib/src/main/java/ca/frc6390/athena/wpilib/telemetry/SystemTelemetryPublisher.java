package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.wpilib.system.SystemStatus;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Publishes roboRIO/JVM health and system-tuning results under {@code /Athena/System}. */
public final class SystemTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/System";
    private final List<NetworkTableEntry> entries = new ArrayList<>();
    private final NetworkTableEntry profile;
    private final NetworkTableEntry target;
    private final NetworkTableEntry pressure;
    private final NetworkTableEntry pressureReason;
    private final NetworkTableEntry totalMemory;
    private final NetworkTableEntry availableMemory;
    private final NetworkTableEntry lowestAvailableMemory;
    private final NetworkTableEntry processResident;
    private final NetworkTableEntry heapUsed;
    private final NetworkTableEntry heapCommitted;
    private final NetworkTableEntry heapMaximum;
    private final NetworkTableEntry nonHeapUsed;
    private final NetworkTableEntry directBuffers;
    private final NetworkTableEntry directBufferGrowth;
    private final NetworkTableEntry estimatedNativeMemory;
    private final NetworkTableEntry nativeMemoryGrowth;
    private final NetworkTableEntry processResidentGrowth;
    private final NetworkTableEntry allocationRate;
    private final NetworkTableEntry memoryTrend;
    private final NetworkTableEntry exhaustionSeconds;
    private final NetworkTableEntry gcCount;
    private final NetworkTableEntry gcTime;
    private final NetworkTableEntry gcLoad;
    private final NetworkTableEntry processCpu;
    private final NetworkTableEntry systemCpu;
    private final NetworkTableEntry liveThreads;
    private final NetworkTableEntry swapTotal;
    private final NetworkTableEntry swapUsed;
    private final NetworkTableEntry swapKind;
    private final NetworkTableEntry pressureTransitions;
    private final NetworkTableEntry tuningComplete;
    private final NetworkTableEntry tuningVerified;
    private final NetworkTableEntry appliedChanges;
    private final NetworkTableEntry failures;
    private final NetworkTableEntry jvmHealthy;
    private final NetworkTableEntry jvmArguments;
    private final NetworkTableEntry gradleRioSnippet;
    private SystemStatus lastStatus;

    public SystemTelemetryPublisher() {
        this(NetworkTableInstance.getDefault());
    }

    SystemTelemetryPublisher(NetworkTableInstance instance) {
        Objects.requireNonNull(instance, "instance");
        profile = entry(instance, "Profile");
        target = entry(instance, "Target");
        pressure = entry(instance, "Pressure/Level");
        pressureReason = entry(instance, "Pressure/Reason");
        pressureTransitions = entry(instance, "Pressure/Transitions");
        totalMemory = entry(instance, "Memory/TotalBytes");
        availableMemory = entry(instance, "Memory/AvailableBytes");
        lowestAvailableMemory = entry(instance, "Memory/LowestAvailableBytes");
        processResident = entry(instance, "Memory/ProcessResidentBytes");
        memoryTrend = entry(instance, "Memory/AvailableTrendBytesPerSecond");
        exhaustionSeconds = entry(instance, "Memory/EstimatedSecondsToExhaustion");
        heapUsed = entry(instance, "JVM/HeapUsedBytes");
        heapCommitted = entry(instance, "JVM/HeapCommittedBytes");
        heapMaximum = entry(instance, "JVM/HeapMaximumBytes");
        nonHeapUsed = entry(instance, "JVM/NonHeapUsedBytes");
        directBuffers = entry(instance, "JVM/DirectBufferBytes");
        directBufferGrowth = entry(instance, "JVM/DirectBufferGrowthBytesPerSecond");
        estimatedNativeMemory = entry(instance, "JVM/EstimatedNativeMemoryBytes");
        nativeMemoryGrowth = entry(instance, "JVM/NativeMemoryGrowthBytesPerSecond");
        processResidentGrowth = entry(instance, "Memory/ProcessResidentGrowthBytesPerSecond");
        allocationRate = entry(instance, "JVM/AllocationRateBytesPerSecond");
        jvmHealthy = entry(instance, "JVM/ConfigurationHealthy");
        jvmArguments = entry(instance, "JVM/RecommendedArguments");
        gradleRioSnippet = entry(instance, "JVM/GradleRioSnippet");
        gcCount = entry(instance, "GC/CollectionCount");
        gcTime = entry(instance, "GC/CollectionTimeMillis");
        gcLoad = entry(instance, "GC/Load");
        processCpu = entry(instance, "CPU/ProcessLoad");
        systemCpu = entry(instance, "CPU/SystemLoad");
        liveThreads = entry(instance, "Threads/LiveCount");
        swapTotal = entry(instance, "Swap/TotalBytes");
        swapUsed = entry(instance, "Swap/UsedBytes");
        swapKind = entry(instance, "Swap/Kind");
        tuningComplete = entry(instance, "Tuning/Complete");
        tuningVerified = entry(instance, "Tuning/Verified");
        appliedChanges = entry(instance, "Tuning/AppliedChanges");
        failures = entry(instance, "Tuning/Failures");
    }

    public void publish(SystemStatus status) {
        if (status == lastStatus) return;
        lastStatus = status;
        profile.setString(status.profile().name());
        target.setString(status.target());
        pressure.setString(status.pressure().name());
        pressureReason.setString(status.pressureReason());
        pressureTransitions.setInteger(status.pressureTransitionCount());
        totalMemory.setInteger(status.totalMemoryBytes());
        availableMemory.setInteger(status.availableMemoryBytes());
        lowestAvailableMemory.setInteger(status.lowestAvailableMemoryBytes());
        processResident.setInteger(status.processResidentBytes());
        memoryTrend.setDouble(status.availableMemoryTrendBytesPerSecond());
        exhaustionSeconds.setDouble(status.estimatedSecondsToExhaustion());
        heapUsed.setInteger(status.heapUsedBytes());
        heapCommitted.setInteger(status.heapCommittedBytes());
        heapMaximum.setInteger(status.heapMaximumBytes());
        nonHeapUsed.setInteger(status.nonHeapUsedBytes());
        directBuffers.setInteger(status.directBufferBytes());
        directBufferGrowth.setDouble(status.directBufferGrowthBytesPerSecond());
        estimatedNativeMemory.setInteger(status.estimatedNativeMemoryBytes());
        nativeMemoryGrowth.setDouble(status.nativeMemoryGrowthBytesPerSecond());
        processResidentGrowth.setDouble(status.processResidentGrowthBytesPerSecond());
        allocationRate.setDouble(status.allocationRateBytesPerSecond());
        jvmHealthy.setBoolean(status.jvmConfigurationHealthy());
        jvmArguments.setStringArray(status.recommendedJvmArguments().toArray(String[]::new));
        gradleRioSnippet.setString(gradleRioSnippet(status.recommendedJvmArguments()));
        gcCount.setInteger(status.garbageCollectionCount());
        gcTime.setInteger(status.garbageCollectionTimeMillis());
        gcLoad.setDouble(status.garbageCollectionLoad());
        processCpu.setDouble(status.processCpuLoad());
        systemCpu.setDouble(status.systemCpuLoad());
        liveThreads.setInteger(status.liveThreadCount());
        swapTotal.setInteger(status.swapTotalBytes());
        swapUsed.setInteger(status.swapUsedBytes());
        swapKind.setString(status.swapKind());
        tuningComplete.setBoolean(status.tuningComplete());
        tuningVerified.setBoolean(status.tuningVerified());
        appliedChanges.setStringArray(status.appliedChanges().toArray(String[]::new));
        failures.setStringArray(status.failures().toArray(String[]::new));
    }

    private static String gradleRioSnippet(List<String> arguments) {
        if (arguments.isEmpty()) return "";
        return "jvmArgs.addAll(" + arguments.stream()
                .map(argument -> "\"" + argument + "\"")
                .collect(java.util.stream.Collectors.joining(", ")) + ")";
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
