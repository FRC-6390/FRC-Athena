package ca.frc6390.athena.wpilib.system;

import java.util.List;

/** Immutable system health and tuning snapshot. Byte values are {@code -1} when unavailable. */
public record SystemStatus(
        SystemTuning.Profile profile,
        String target,
        MemoryPressure pressure,
        String pressureReason,
        long totalMemoryBytes,
        long availableMemoryBytes,
        long lowestAvailableMemoryBytes,
        long processResidentBytes,
        long heapUsedBytes,
        long heapCommittedBytes,
        long heapMaximumBytes,
        long nonHeapUsedBytes,
        long directBufferBytes,
        double directBufferGrowthBytesPerSecond,
        long estimatedNativeMemoryBytes,
        double nativeMemoryGrowthBytesPerSecond,
        double processResidentGrowthBytesPerSecond,
        double allocationRateBytesPerSecond,
        double availableMemoryTrendBytesPerSecond,
        double estimatedSecondsToExhaustion,
        long garbageCollectionCount,
        long garbageCollectionTimeMillis,
        double garbageCollectionLoad,
        double processCpuLoad,
        double systemCpuLoad,
        int liveThreadCount,
        long swapTotalBytes,
        long swapUsedBytes,
        String swapKind,
        long pressureTransitionCount,
        boolean tuningComplete,
        boolean tuningVerified,
        boolean jvmConfigurationHealthy,
        List<String> recommendedJvmArguments,
        List<String> appliedChanges,
        List<String> failures) {

    public SystemStatus {
        recommendedJvmArguments = List.copyOf(recommendedJvmArguments);
        appliedChanges = List.copyOf(appliedChanges);
        failures = List.copyOf(failures);
    }
}
