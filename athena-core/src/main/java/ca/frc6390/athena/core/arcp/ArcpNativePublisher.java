package ca.frc6390.athena.core.arcp;

import ca.frc6390.athena.arcp.ArcpRuntime;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Native ARCP signal registry/publisher used by Athena runtime code.
 *
 * <p>This avoids any dependency on NetworkTables topic mirroring. Signals are declared and pushed
 * directly through the ARCP runtime.</p>
 */
public final class ArcpNativePublisher {
    private static final int MAX_SIGNAL_ID = 0xFFFF;

    private final ArcpRuntime runtime;
    private final Map<SignalKey, Integer> signalIds;
    private int nextSignalId;

    public ArcpNativePublisher(ArcpRuntime runtime) {
        this.runtime = Objects.requireNonNull(runtime, "runtime");
        this.signalIds = new HashMap<>();
        this.nextSignalId = 1;
    }

    public void publishBoolean(String path, boolean value) {
        int signalId = ensureSignal(path, ArcpRuntime.SignalType.BOOL);
        runtime.publishBoolean(signalId, value);
    }

    public void publishDouble(String path, double value) {
        int signalId = ensureSignal(path, ArcpRuntime.SignalType.F64);
        runtime.publishDouble(signalId, value);
    }

    public void publishI64(String path, long value) {
        int signalId = ensureSignal(path, ArcpRuntime.SignalType.I64);
        runtime.publishI64(signalId, value);
    }

    public void publishString(String path, String value) {
        int signalId = ensureSignal(path, ArcpRuntime.SignalType.STRING);
        runtime.publishString(signalId, value != null ? value : "");
    }

    private int ensureSignal(String rawPath, ArcpRuntime.SignalType signalType) {
        String path = normalizePath(rawPath);
        SignalKey key = new SignalKey(path, signalType);
        synchronized (signalIds) {
            Integer existing = signalIds.get(key);
            if (existing != null) {
                return existing.intValue();
            }
            int signalId = allocateSignalId();
            runtime.registerSignal(
                    signalId,
                    signalType,
                    ArcpRuntime.SignalKind.TELEMETRY,
                    ArcpRuntime.SignalAccess.OBSERVE,
                    ArcpRuntime.SignalPolicy.ON_CHANGE,
                    ArcpRuntime.SignalDurability.VOLATILE,
                    path);
            signalIds.put(key, signalId);
            return signalId;
        }
    }

    private int allocateSignalId() {
        if (nextSignalId > MAX_SIGNAL_ID) {
            throw new IllegalStateException("ARCP signal ID space exhausted (max=" + MAX_SIGNAL_ID + ")");
        }
        return nextSignalId++;
    }

    private static String normalizePath(String rawPath) {
        if (rawPath == null) {
            return "Athena/unknown";
        }
        String trimmed = rawPath.trim();
        if (trimmed.isEmpty()) {
            return "Athena/unknown";
        }
        if (trimmed.startsWith("/")) {
            return trimmed.substring(1);
        }
        return trimmed;
    }

    private record SignalKey(String path, ArcpRuntime.SignalType signalType) {}
}

