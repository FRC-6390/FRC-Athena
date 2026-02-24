package ca.frc6390.athena.arcp;

import java.nio.ByteBuffer;

final class ArcpNative {
    static {
        System.loadLibrary("arcp_jni");
    }

    private ArcpNative() {
    }

    static native long runtimeCreate();

    static native void runtimeDestroy(long handle);

    static native int runtimeStart(long handle);

    static native int runtimeStop(long handle);

    static native int controlPort(long handle);

    static native int realtimePort(long handle);

    static native int registerSignal(
            long handle,
            int signalId,
            int signalType,
            int signalKind,
            int signalAccess,
            int signalPolicy,
            int signalDurability,
            String path);

    static native int publishBoolean(long handle, int signalId, boolean value);

    static native int publishI64(long handle, int signalId, long value);

    static native int publishF64(long handle, int signalId, double value);

    static native int publishString(long handle, int signalId, String value);

    static native int publishBooleanArray(long handle, int signalId, boolean[] values);

    static native int publishI64Array(long handle, int signalId, long[] values);

    static native int publishF64Array(long handle, int signalId, double[] values);

    static native int publishStringArray(long handle, int signalId, String[] values);

    static native int pollEvents(long handle, ByteBuffer outBuffer, int outCapacity);
}
