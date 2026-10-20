package ca.frc6390.athena.arcp;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Locale;

/**
 * JNI declarations for the Rust ARCP server (crate {@code athena-arcp/rust/crates/arcp-jni}).
 * Nothing here encodes the ARCP protocol; see {@link ArcpMarshal} for the buffer layout.
 *
 * <p>Loading order: the {@code athena.arcp.nativePath} system property (absolute path to the
 * library), then the bundled {@code /native/<os>-<arch>/<library>} resource, then
 * {@link System#loadLibrary(String)}.</p>
 */
final class ArcpNative {

    static final String LIBRARY_BASE = "arcp_jni";

    private static Throwable lastBundleFailure;

    static {
        load();
    }

    private ArcpNative() {
    }

    private static void load() {
        String override = System.getProperty("athena.arcp.nativePath");
        if (override != null && !override.isBlank()) {
            System.load(override);
            return;
        }
        String resource = bundledResourcePath();
        if (resource != null) {
            try {
                System.load(extract(resource));
                return;
            } catch (IOException | UnsatisfiedLinkError ex) {
                // Fall through to the system library path, keep the reason for the error message.
                lastBundleFailure = ex;
            }
        }
        try {
            System.loadLibrary(LIBRARY_BASE);
        } catch (UnsatisfiedLinkError ex) {
            UnsatisfiedLinkError error = new UnsatisfiedLinkError("ARCP native library not available: resource="
                    + resource + ", bundleFailure=" + lastBundleFailure + ", systemLibrary=" + ex.getMessage());
            throw error;
        }
    }

    static String bundledResourcePath() {
        String os = System.getProperty("os.name", "").toLowerCase(Locale.ROOT);
        String arch = normalizeArch(System.getProperty("os.arch", ""));
        if (arch == null) {
            return null;
        }
        if (os.contains("linux")) {
            return "/native/linux-" + arch + "/lib" + LIBRARY_BASE + ".so";
        }
        if (os.contains("mac")) {
            return "/native/macos-" + arch + "/lib" + LIBRARY_BASE + ".dylib";
        }
        if (os.contains("windows")) {
            return "/native/windows-" + arch + "/" + LIBRARY_BASE + ".dll";
        }
        return null;
    }

    static String normalizeArch(String raw) {
        String arch = raw == null ? "" : raw.toLowerCase(Locale.ROOT);
        if (arch.equals("x86_64") || arch.equals("amd64")) {
            return "x86_64";
        }
        if (arch.equals("aarch64") || arch.equals("arm64")) {
            return "aarch64";
        }
        if (arch.startsWith("armv7") || arch.equals("arm")) {
            return "arm32";
        }
        return null;
    }

    private static String extract(String resource) throws IOException {
        try (InputStream in = ArcpNative.class.getResourceAsStream(resource)) {
            if (in == null) {
                throw new IOException("missing resource " + resource);
            }
            String fileName = resource.substring(resource.lastIndexOf('/') + 1);
            Path dir = Files.createTempDirectory("athena-arcp-native");
            Path target = dir.resolve(fileName);
            Files.copy(in, target, StandardCopyOption.REPLACE_EXISTING);
            target.toFile().deleteOnExit();
            dir.toFile().deleteOnExit();
            return target.toAbsolutePath().toString();
        }
    }

    /**
     * @param fileCategories {@code name<TAB>absoluteDirectory} lines, one per category, or null
     */
    static native long create(int controlPort, int realtimePort, String fileRoot, long budgetBytesPerSecond, int mtu,
            long keyframeIntervalMicros, String fileCategories);

    static native void destroy(long handle);

    static native int controlPort(long handle);

    static native int realtimePort(long handle);

    static native void publishDictionary(long handle, ByteBuffer buffer, int length);

    static native void publishCycle(long handle, long timestampMicros, ByteBuffer buffer, int length);

    static native void marker(long handle, long timestampMicros, int kind, long signalId, String message);

    /** Sends raw replay record envelopes to sessions that opted in with ReplaySubscribe. */
    static native void publishRecords(long handle, ByteBuffer buffer, int length);

    /** Returns bytes written, or the negative size needed when the buffer is too small. */
    static native int poll(long handle, ByteBuffer buffer, int capacity);

    static native void reply(long handle, ByteBuffer buffer, int length);

    /** {@code [sessions, subscriptions, bytesLastWindow, throttledCount]}. */
    static native long[] status(long handle);

    /** Throttled subscriptions, one per line. */
    static native String throttled(long handle);

    /** Changes whenever the set of subscribed signals may have changed. */
    static native long subscriptionGeneration(long handle);

    /** Sorted ids of the signals matched by at least one subscription. */
    static native long[] subscribedSignalIds(long handle);

    /** Number of connected control sessions. */
    static native int sessionCount(long handle);
}
