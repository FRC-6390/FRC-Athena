package ca.frc6390.athena.arcp;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Locale;

final class ArcpNative {
    static {
        loadNativeLibrary();
    }

    private ArcpNative() {
    }

    private static synchronized void loadNativeLibrary() {
        String resourcePath = resolveBundledNativePath();
        if (resourcePath != null) {
            try {
                loadBundled(resourcePath);
                return;
            } catch (IOException | RuntimeException ex) {
                // Fallback below: system library path.
            }
        }
        System.loadLibrary("arcp_jni");
    }

    private static String resolveBundledNativePath() {
        String os = System.getProperty("os.name", "").toLowerCase(Locale.ROOT);
        String arch = normalizeArch(System.getProperty("os.arch", ""));
        if (arch == null) {
            return null;
        }

        if (os.contains("linux")) {
            return "/native/linux-" + arch + "/libarcp_jni.so";
        }
        if (os.contains("mac") || os.contains("darwin")) {
            return "/native/macos-" + arch + "/libarcp_jni.dylib";
        }
        if (os.contains("win")) {
            return "/native/windows-" + arch + "/arcp_jni.dll";
        }
        return null;
    }

    private static String normalizeArch(String rawArch) {
        String arch = rawArch == null ? "" : rawArch.toLowerCase(Locale.ROOT).trim();
        if (arch.equals("amd64") || arch.equals("x86_64")) {
            return "x86_64";
        }
        if (arch.equals("aarch64") || arch.equals("arm64")) {
            return "aarch64";
        }
        if (arch.equals("arm")
                || arch.startsWith("armv7")
                || arch.startsWith("armeabi")) {
            return "arm32";
        }
        return null;
    }

    private static void loadBundled(String resourcePath) throws IOException {
        try (InputStream input = ArcpNative.class.getResourceAsStream(resourcePath)) {
            if (input == null) {
                throw new IOException("Missing bundled ARCP JNI binary: " + resourcePath);
            }

            String fileName = resourcePath.substring(resourcePath.lastIndexOf('/') + 1);
            String suffix = fileName.contains(".")
                    ? fileName.substring(fileName.lastIndexOf('.'))
                    : ".bin";

            Path tempDir = Path.of(System.getProperty("java.io.tmpdir"), "athena-arcp-native");
            Files.createDirectories(tempDir);

            Path extracted = Files.createTempFile(tempDir, "arcp_jni-", suffix);
            Files.copy(input, extracted, StandardCopyOption.REPLACE_EXISTING);
            extracted.toFile().deleteOnExit();
            System.load(extracted.toAbsolutePath().toString());
        }
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

    static native int saveLayout(long handle, String name, String layoutJson);

    static native String loadLayout(long handle, String name);

    static native String[] listLayouts(long handle);
}
