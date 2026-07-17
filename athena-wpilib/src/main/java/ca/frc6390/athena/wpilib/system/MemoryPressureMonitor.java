package ca.frc6390.athena.wpilib.system;

final class MemoryPressureMonitor {
    private static final long MIB = 1024L * 1024L;
    private static final long IMMEDIATE_CRITICAL_BYTES = 8L * MIB;
    private final SystemTuning tuning;
    private MemoryPressure pressure = MemoryPressure.NORMAL;
    private int warningSamples;
    private int criticalSamples;
    private int recoverySamples;
    private long transitions;
    private long lowestAvailable = Long.MAX_VALUE;
    private double availableTrend;
    private String reason = "Memory headroom is normal";

    MemoryPressureMonitor(SystemTuning tuning) {
        this.tuning = tuning;
    }

    Result update(SystemAccess.Memory memory, double trendBytesPerSecond, double secondsToExhaustion, double gcLoad) {
        if (memory.available() >= 0) lowestAvailable = Math.min(lowestAvailable, memory.available());
        availableTrend = Double.isFinite(trendBytesPerSecond) ? trendBytesPerSecond : 0.0;
        Candidate candidate = candidate(memory, secondsToExhaustion, gcLoad);
        if (candidate.pressure == MemoryPressure.CRITICAL) {
            criticalSamples++;
            warningSamples = 0;
            recoverySamples = 0;
            if (memory.available() <= IMMEDIATE_CRITICAL_BYTES
                    || criticalSamples >= Math.max(1, tuning.escalationSamples() - 1)) {
                transition(MemoryPressure.CRITICAL, candidate.reason);
            }
        } else if (candidate.pressure == MemoryPressure.WARNING) {
            warningSamples++;
            criticalSamples = 0;
            if (pressure == MemoryPressure.CRITICAL) {
                recoverToward(MemoryPressure.WARNING, candidate.reason);
            } else {
                recoverySamples = 0;
                if (warningSamples >= tuning.escalationSamples()) {
                    transition(MemoryPressure.WARNING, candidate.reason);
                }
            }
        } else {
            warningSamples = 0;
            criticalSamples = 0;
            if (pressure != MemoryPressure.NORMAL) recoverToward(MemoryPressure.NORMAL, candidate.reason);
            else reason = candidate.reason;
        }
        return result(secondsToExhaustion);
    }

    private Candidate candidate(SystemAccess.Memory memory, double secondsToExhaustion, double gcLoad) {
        if (memory.total() <= 0 || memory.available() < 0) {
            return new Candidate(MemoryPressure.NORMAL, "Operating-system memory data is unavailable");
        }
        double availableFraction = (double) memory.available() / memory.total();
        if (memory.available() <= IMMEDIATE_CRITICAL_BYTES) {
            return new Candidate(MemoryPressure.CRITICAL, "Available RAM is below 8 MiB");
        }
        if (availableFraction <= tuning.criticalAvailableFraction()) {
            return new Candidate(MemoryPressure.CRITICAL, "Available RAM crossed the critical threshold");
        }
        if (Double.isFinite(secondsToExhaustion)
                && secondsToExhaustion >= 0.0 && secondsToExhaustion <= 10.0) {
            return new Candidate(MemoryPressure.CRITICAL, "Available RAM is falling with less than 10 seconds remaining");
        }
        if (availableFraction <= tuning.warningAvailableFraction()) {
            return new Candidate(MemoryPressure.WARNING, "Available RAM crossed the warning threshold");
        }
        if (Double.isFinite(secondsToExhaustion)
                && secondsToExhaustion >= 0.0 && secondsToExhaustion <= 30.0) {
            return new Candidate(MemoryPressure.WARNING, "Available RAM is falling with less than 30 seconds remaining");
        }
        if (gcLoad >= 0.25 && availableFraction <= tuning.warningAvailableFraction() * 1.5) {
            return new Candidate(MemoryPressure.WARNING, "Garbage collection is consuming at least 25% of runtime");
        }
        return new Candidate(MemoryPressure.NORMAL, "Memory headroom is normal");
    }

    private void recoverToward(MemoryPressure target, String nextReason) {
        recoverySamples++;
        if (recoverySamples < tuning.recoverySamples()) return;
        transition(target, nextReason);
        recoverySamples = 0;
    }

    private void transition(MemoryPressure target, String nextReason) {
        if (pressure != target) {
            pressure = target;
            transitions++;
        }
        reason = nextReason;
    }

    private Result result(double secondsToExhaustion) {
        return new Result(
                pressure,
                reason,
                lowestAvailable == Long.MAX_VALUE ? -1 : lowestAvailable,
                availableTrend,
                secondsToExhaustion,
                transitions);
    }

    private record Candidate(MemoryPressure pressure, String reason) { }

    record Result(
            MemoryPressure pressure,
            String reason,
            long lowestAvailableBytes,
            double availableTrendBytesPerSecond,
            double estimatedSecondsToExhaustion,
            long transitions) { }
}
