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

    static SystemStatus pending(SystemTuning.Profile profile, boolean tuningComplete) {
        return new SystemStatus(profile, "Unknown", MemoryPressure.NORMAL, "Awaiting background system sample",
                -1, -1, -1, -1,
                -1, -1, -1, -1, -1,
                0, -1, 0, 0, -1, 0, -1,
                -1, -1, 0, -1, -1, -1,
                -1, -1, "UNKNOWN", 0,
                tuningComplete, tuningComplete, false, List.of(), List.of(), List.of());
    }

    SystemStatus withTuning(boolean complete, List<String> changes, List<String> errors) {
        return new SystemStatus(profile, target, pressure, pressureReason,
                totalMemoryBytes, availableMemoryBytes, lowestAvailableMemoryBytes, processResidentBytes,
                heapUsedBytes, heapCommittedBytes, heapMaximumBytes, nonHeapUsedBytes, directBufferBytes,
                directBufferGrowthBytesPerSecond, estimatedNativeMemoryBytes, nativeMemoryGrowthBytesPerSecond,
                processResidentGrowthBytesPerSecond, allocationRateBytesPerSecond, availableMemoryTrendBytesPerSecond,
                estimatedSecondsToExhaustion, garbageCollectionCount, garbageCollectionTimeMillis, garbageCollectionLoad,
                processCpuLoad, systemCpuLoad, liveThreadCount, swapTotalBytes, swapUsedBytes, swapKind,
                pressureTransitionCount, complete, complete && errors.isEmpty(), jvmConfigurationHealthy,
                recommendedJvmArguments, changes, errors);
    }
}
