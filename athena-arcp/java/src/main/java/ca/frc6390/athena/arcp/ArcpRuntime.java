package ca.frc6390.athena.arcp;

import java.nio.ByteBuffer;

/**
 * Thin Java facade over the Rust ARCP runtime.
 */
public final class ArcpRuntime implements AutoCloseable {
    public enum SignalType {
        BOOL(1),
        I64(2),
        F64(3),
        STRING(4),
        BOOL_ARRAY(5),
        I64_ARRAY(6),
        F64_ARRAY(7),
        STRING_ARRAY(8);

        private final int wireValue;

        SignalType(int wireValue) {
            this.wireValue = wireValue;
        }

        int wireValue() {
            return wireValue;
        }
    }

    public enum SignalKind {
        TELEMETRY(1),
        COMMAND(2);

        private final int wireValue;

        SignalKind(int wireValue) {
            this.wireValue = wireValue;
        }

        int wireValue() {
            return wireValue;
        }
    }

    public enum SignalAccess {
        OBSERVE(1),
        WRITE(2),
        INVOKE(3);

        private final int wireValue;

        SignalAccess(int wireValue) {
            this.wireValue = wireValue;
        }

        int wireValue() {
            return wireValue;
        }
    }

    public enum SignalPolicy {
        HIGH_RATE(1),
        ON_CHANGE(2),
        SAMPLED(3),
        SNAPSHOT_ONLY(4);

        private final int wireValue;

        SignalPolicy(int wireValue) {
            this.wireValue = wireValue;
        }

        int wireValue() {
            return wireValue;
        }
    }

    public enum SignalDurability {
        VOLATILE(1),
        RETAINED(2),
        PERSISTENT(3);

        private final int wireValue;

        SignalDurability(int wireValue) {
            this.wireValue = wireValue;
        }

        int wireValue() {
            return wireValue;
        }
    }

    private final long handle;
    private boolean running;

    private ArcpRuntime(long handle) {
        this.handle = handle;
    }

    public static ArcpRuntime create() {
        long handle = ArcpNative.runtimeCreate();
        if (handle == 0L) {
            throw new IllegalStateException("Failed to create ARCP runtime");
        }
        return new ArcpRuntime(handle);
    }

    public void start() {
        int rc = ArcpNative.runtimeStart(handle);
        if (rc != 0) {
            throw new IllegalStateException("ARCP runtime start failed: " + rc);
        }
        running = true;
    }

    public void stop() {
        int rc = ArcpNative.runtimeStop(handle);
        if (rc != 0) {
            throw new IllegalStateException("ARCP runtime stop failed: " + rc);
        }
        running = false;
    }

    public int controlPort() {
        return ArcpNative.controlPort(handle);
    }

    public int realtimePort() {
        return ArcpNative.realtimePort(handle);
    }

    public void registerSignal(
            int signalId,
            SignalType signalType,
            SignalKind signalKind,
            SignalAccess signalAccess,
            SignalPolicy signalPolicy,
            SignalDurability signalDurability,
            String path) {
        if (signalType == null || signalKind == null || signalAccess == null || signalPolicy == null
                || signalDurability == null || path == null || path.isBlank()) {
            throw new IllegalArgumentException("signalType, signalKind, signalAccess, signalPolicy, signalDurability, and path must be provided");
        }
        int rc = ArcpNative.registerSignal(
                handle,
                signalId,
                signalType.wireValue(),
                signalKind.wireValue(),
                signalAccess.wireValue(),
                signalPolicy.wireValue(),
                signalDurability.wireValue(),
                path);
        if (rc != 0) {
            throw new IllegalStateException("ARCP registerSignal failed: " + rc);
        }
    }

    public void publishBoolean(int signalId, boolean value) {
        int rc = ArcpNative.publishBoolean(handle, signalId, value);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishBoolean failed: " + rc);
        }
    }

    public void publishI64(int signalId, long value) {
        int rc = ArcpNative.publishI64(handle, signalId, value);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishI64 failed: " + rc);
        }
    }

    public void publishDouble(int signalId, double value) {
        int rc = ArcpNative.publishF64(handle, signalId, value);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishDouble failed: " + rc);
        }
    }

    public void publishString(int signalId, String value) {
        if (value == null) {
            throw new IllegalArgumentException("value must not be null");
        }
        int rc = ArcpNative.publishString(handle, signalId, value);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishString failed: " + rc);
        }
    }

    public void publishBooleanArray(int signalId, boolean[] values) {
        if (values == null) {
            throw new IllegalArgumentException("values must not be null");
        }
        int rc = ArcpNative.publishBooleanArray(handle, signalId, values);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishBooleanArray failed: " + rc);
        }
    }

    public void publishI64Array(int signalId, long[] values) {
        if (values == null) {
            throw new IllegalArgumentException("values must not be null");
        }
        int rc = ArcpNative.publishI64Array(handle, signalId, values);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishI64Array failed: " + rc);
        }
    }

    public void publishF64Array(int signalId, double[] values) {
        if (values == null) {
            throw new IllegalArgumentException("values must not be null");
        }
        int rc = ArcpNative.publishF64Array(handle, signalId, values);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishF64Array failed: " + rc);
        }
    }

    public void publishStringArray(int signalId, String[] values) {
        if (values == null) {
            throw new IllegalArgumentException("values must not be null");
        }
        int rc = ArcpNative.publishStringArray(handle, signalId, values);
        if (rc != 0) {
            throw new IllegalStateException("ARCP publishStringArray failed: " + rc);
        }
    }

    public int pollEvents(ByteBuffer target) {
        if (target == null || !target.isDirect()) {
            throw new IllegalArgumentException("target must be a direct ByteBuffer");
        }
        return ArcpNative.pollEvents(handle, target, target.capacity());
    }

    public void saveLayout(String name, String layoutJson) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("name must be non-empty");
        }
        if (layoutJson == null || layoutJson.isBlank()) {
            throw new IllegalArgumentException("layoutJson must be non-empty");
        }
        int rc = ArcpNative.saveLayout(handle, name, layoutJson);
        if (rc != 0) {
            throw new IllegalStateException("ARCP saveLayout failed: " + rc);
        }
    }

    public String loadLayout(String name) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("name must be non-empty");
        }
        String payload = ArcpNative.loadLayout(handle, name);
        if (payload == null) {
            throw new IllegalStateException("ARCP loadLayout failed for profile: " + name);
        }
        return payload;
    }

    public String[] listLayouts() {
        String[] values = ArcpNative.listLayouts(handle);
        return values != null ? values : new String[0];
    }

    public boolean isRunning() {
        return running;
    }

    @Override
    public void close() {
        if (running) {
            stop();
        }
        ArcpNative.runtimeDestroy(handle);
    }
}
