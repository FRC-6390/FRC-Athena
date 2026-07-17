package ca.frc6390.athena.wpilib.system;

import java.util.List;

/** Immutable system health and tuning snapshot. Byte values are {@code -1} when unavailable. */
public record SystemStatus(
        SystemTuning.Profile profile,
        String target,
        MemoryPressure pressure,
        long totalMemoryBytes,
        long availableMemoryBytes,
        long processResidentBytes,
        long heapUsedBytes,
        long heapMaximumBytes,
        long directBufferBytes,
        long swapTotalBytes,
        long swapUsedBytes,
        boolean tuningComplete,
        List<String> appliedChanges,
        List<String> failures) {

    public SystemStatus {
        appliedChanges = List.copyOf(appliedChanges);
        failures = List.copyOf(failures);
    }
}
