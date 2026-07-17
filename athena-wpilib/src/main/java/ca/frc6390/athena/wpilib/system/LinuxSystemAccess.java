package ca.frc6390.athena.wpilib.system;

import java.io.IOException;
import java.io.OutputStream;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileStore;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

final class LinuxSystemAccess implements SystemAccess {
    private static final Path SWAP_FILE = Path.of("/home/lvuser/athena/athena-swap.img");
    private static final Path ZRAM_DEVICE = Path.of("/dev/zram0");
    private static final Path ZRAM_DISKSIZE = Path.of("/sys/block/zram0/disksize");
    private static final Path ZRAM_RESET = Path.of("/sys/block/zram0/reset");
    private static final String WEB_SERVER = "/usr/local/natinst/share/NIWebServer/SystemWebServer";
    private static final Duration COMMAND_TIMEOUT = Duration.ofSeconds(10);
    private final boolean realRobot;
    private String detectedTarget;
    private volatile List<String> cachedLoopDevices;

    LinuxSystemAccess(boolean realRobot) {
        this.realRobot = realRobot;
    }

    @Override public boolean realRobot() { return realRobot; }

    @Override
    public boolean linux() {
        return System.getProperty("os.name", "").toLowerCase(Locale.ROOT).contains("linux");
    }

    @Override
    public String target() {
        if (detectedTarget != null) return detectedTarget;
        String model = readFirst(
                Path.of("/proc/device-tree/model"),
                Path.of("/sys/firmware/devicetree/base/model"));
        detectedTarget = !model.isBlank()
                ? model.replace("\0", "").trim()
                : realRobot ? "Unknown Linux robot" : "Simulation";
        return detectedTarget;
    }

    @Override
    public Memory memory() {
        long total = -1;
        long available = -1;
        long swapTotal = -1;
        long swapFree = -1;
        for (String line : readLines(Path.of("/proc/meminfo"))) {
            if (line.startsWith("MemTotal:")) total = kib(line);
            else if (line.startsWith("MemAvailable:")) available = kib(line);
            else if (line.startsWith("SwapTotal:")) swapTotal = kib(line);
            else if (line.startsWith("SwapFree:")) swapFree = kib(line);
        }
        long rss = -1;
        for (String line : readLines(Path.of("/proc/self/status"))) {
            if (line.startsWith("VmRSS:")) {
                rss = kib(line);
                break;
            }
        }
        long swapUsed = swapTotal >= 0 && swapFree >= 0 ? Math.max(0, swapTotal - swapFree) : -1;
        return new Memory(total, available, rss, swapTotal, swapUsed);
    }

    @Override
    public String swapKind() {
        if (!linux()) return "NONE";
        boolean zram = false;
        boolean file = false;
        List<String> swaps = activeSwaps();
        List<String> loops = attachedLoopDevices();
        for (String line : swaps) {
            zram |= line.toLowerCase(Locale.ROOT).contains("zram");
            file |= line.contains(SWAP_FILE.toString()) || loops.stream().anyMatch(line::startsWith);
        }
        if (zram && file) return "ZRAM_AND_FILE";
        if (zram) return "ZRAM";
        if (file) return "FILE";
        return swaps.isEmpty() ? "NONE" : "EXTERNAL";
    }

    @Override
    public String readSysctl(String key) {
        if (!allowedSysctl(key)) return "";
        return readFirst(Path.of("/proc/sys/" + key.replace('.', '/'))).trim();
    }

    @Override
    public boolean niWebServerRunning() {
        boolean visible = ProcessHandle.allProcesses().anyMatch(process -> process.info().command().map(command ->
                command.equals(WEB_SERVER)
                        || command.endsWith("/SystemWebServer")
                        || command.endsWith("/NIWebServiceContainer")).orElse(false));
        if (visible) return true;
        return command("/usr/bin/pgrep", "-f", "^" + WEB_SERVER).success;
    }

    @Override
    public boolean zramSupported() {
        return Files.exists(ZRAM_DISKSIZE)
                || Files.exists(Path.of("/sys/class/zram-control/hot_add"))
                || Files.isExecutable(Path.of("/sbin/modprobe"));
    }

    @Override public boolean hasActiveZram() {
        if (!linux()) return false;
        return activeSwaps().stream().anyMatch(line -> line.toLowerCase(Locale.ROOT).contains("zram"));
    }

    @Override
    public boolean hasActiveSwapFile() {
        if (!linux()) return false;
        if (activeSwaps().stream().anyMatch(line -> line.startsWith(SWAP_FILE.toString()))) return true;
        List<String> loops = attachedLoopDevices();
        return activeSwaps().stream().anyMatch(line -> loops.stream().anyMatch(line::startsWith));
    }

    @Override
    public long usableSwapDiskBytes() {
        try {
            Files.createDirectories(SWAP_FILE.getParent());
            FileStore store = Files.getFileStore(SWAP_FILE.getParent());
            return store.getUsableSpace();
        } catch (IOException ignored) {
            return -1;
        }
    }

    @Override
    public Result stopNiWebServer() {
        if (!niWebServerRunning()) return Result.success("NI web services already stopped");
        command("sudo", "-n", "/usr/bin/pkill", "-TERM", "-f", "^" + WEB_SERVER);
        command("sudo", "-n", "/usr/bin/pkill", "-TERM", "-f",
                "^.*/NIWebServiceContainer");
        return waitForWebServer(false)
                ? Result.success("NI web services stopped and verified")
                : Result.failure("NI web services remained active after stop request");
    }

    @Override
    public Result startNiWebServer() {
        if (niWebServerRunning()) return Result.success("NI web services already running");
        CommandResult result = command(
                "sudo", "-n", "/sbin/start-stop-daemon", "-S", "-b", "-x", WEB_SERVER,
                "--", "-timeout", "50", "-child-timeout", "20", "-system");
        if (!result.success) return Result.failure(result.detail);
        return waitForWebServer(true)
                ? Result.success("NI web services restored and verified")
                : Result.failure("NI web services did not start");
    }

    @Override
    public Result setSysctl(String key, int value) {
        if (!allowedSysctl(key)) return Result.failure("Unsupported sysctl " + key);
        CommandResult result = command("sudo", "-n", "/sbin/sysctl", "-w", key + "=" + value);
        if (!result.success) return Result.failure(result.detail);
        String actual = readSysctl(key);
        return Integer.toString(value).equals(actual)
                ? Result.success(key + "=" + value + " verified")
                : Result.failure("readback for " + key + " was '" + actual + "'");
    }

    @Override
    public Result enableZram(long bytes) {
        if (hasActiveZram()) return Result.success("Existing zram swap is active");
        if (!Files.exists(ZRAM_DISKSIZE)) {
            CommandResult module = command("sudo", "-n", "/sbin/modprobe", "zram", "num_devices=1");
            if (!module.success || !Files.exists(ZRAM_DISKSIZE)) {
                return Result.failure("zram kernel support is unavailable: " + module.detail);
            }
        }
        Result reset = writePrivilegedNoReadback(ZRAM_RESET, "1");
        if (!reset.success()) return reset;
        Result size = writePrivileged(ZRAM_DISKSIZE, Long.toString(bytes));
        if (!size.success()) return size;
        CommandResult format = command("sudo", "-n", "/sbin/mkswap", ZRAM_DEVICE.toString());
        if (!format.success) {
            disableZram();
            return Result.failure("zram mkswap: " + format.detail);
        }
        CommandResult enable = command(
                "sudo", "-n", "/sbin/swapon", "-p", "100", ZRAM_DEVICE.toString());
        if (!enable.success) {
            disableZram();
            return Result.failure("zram swapon: " + enable.detail);
        }
        return hasActiveZram()
                ? Result.success("Enabled and verified " + bytes / (1024 * 1024) + " MiB zram")
                : Result.failure("zram swapon succeeded but /proc/swaps has no zram device");
    }

    @Override
    public Result disableZram() {
        for (String line : activeSwaps()) {
            String device = firstField(line);
            if (device.toLowerCase(Locale.ROOT).contains("zram")) {
                CommandResult off = command("sudo", "-n", "/sbin/swapoff", device);
                if (!off.success) return Result.failure("zram swapoff: " + off.detail);
            }
        }
        if (Files.exists(ZRAM_RESET)) {
            Result reset = writePrivilegedNoReadback(ZRAM_RESET, "1");
            if (!reset.success()) return reset;
        }
        return !hasActiveZram()
                ? Result.success("Athena zram disabled and verified")
                : Result.failure("zram remains active");
    }

    @Override
    public Result enableSwapFile(long bytes, long minimumFreeDiskBytes) {
        if (hasActiveSwapFile()) return Result.success("Athena fallback swap is already active");
        long usable = usableSwapDiskBytes();
        if (usable < 0 || usable - bytes < minimumFreeDiskBytes) {
            return Result.failure("Insufficient disk headroom for fallback swap: usable=" + usable
                    + ", required=" + (bytes + minimumFreeDiskBytes));
        }
        try {
            Files.createDirectories(SWAP_FILE.getParent());
            try (RandomAccessFile file = new RandomAccessFile(SWAP_FILE.toFile(), "rw")) {
                file.setLength(bytes);
            }
            try {
                Files.setPosixFilePermissions(SWAP_FILE, PosixFilePermissions.fromString("rw-------"));
            } catch (UnsupportedOperationException ignored) {
                // roboRIO is POSIX; this only protects non-POSIX test hosts.
            }
        } catch (IOException exception) {
            return Result.failure("Cannot create swap file: " + exception.getMessage());
        }
        CommandResult format = command("sudo", "-n", "/sbin/mkswap", SWAP_FILE.toString());
        if (!format.success) {
            deleteSwapFileQuietly();
            return Result.failure("mkswap: " + format.detail);
        }
        CommandResult direct = command("sudo", "-n", "/sbin/swapon", "-p", "10", SWAP_FILE.toString());
        if (!direct.success) {
            CommandResult attach = command("sudo", "-n", "/sbin/losetup", "-f", "--show", SWAP_FILE.toString());
            if (!attach.success || attach.detail.isBlank()) {
                deleteSwapFileQuietly();
                return Result.failure("direct swapon failed and loop attach failed: " + direct.detail
                        + "; " + attach.detail);
            }
            String loop = attach.detail.lines().findFirst().orElse("").trim();
            cachedLoopDevices = List.of(loop);
            CommandResult loopFormat = command("sudo", "-n", "/sbin/mkswap", loop);
            if (!loopFormat.success) {
                cleanupFailedLoop(loop);
                return Result.failure("loop mkswap: " + loopFormat.detail);
            }
            CommandResult loopEnable = command("sudo", "-n", "/sbin/swapon", "-p", "10", loop);
            if (!loopEnable.success) {
                cleanupFailedLoop(loop);
                return Result.failure("loop swapon: " + loopEnable.detail);
            }
        }
        return hasActiveSwapFile()
                ? Result.success("Enabled and verified " + bytes / (1024 * 1024) + " MiB fallback swap")
                : Result.failure("swapon succeeded but fallback swap is absent from /proc/swaps");
    }

    @Override
    public Result disableSwapFile() {
        List<String> loops = attachedLoopDevices();
        for (String line : activeSwaps()) {
            String device = firstField(line);
            if (device.equals(SWAP_FILE.toString()) || loops.contains(device)) {
                CommandResult off = command("sudo", "-n", "/sbin/swapoff", device);
                if (!off.success) return Result.failure("fallback swapoff: " + off.detail);
            }
        }
        for (String loop : loops) {
            CommandResult detach = command("sudo", "-n", "/sbin/losetup", "-d", loop);
            if (!detach.success) return Result.failure("loop detach: " + detach.detail);
        }
        cachedLoopDevices = List.of();
        try {
            Files.deleteIfExists(SWAP_FILE);
        } catch (IOException exception) {
            return Result.failure("Cannot remove fallback swap file: " + exception.getMessage());
        }
        return !hasActiveSwapFile()
                ? Result.success("Athena fallback swap removed and verified")
                : Result.failure("Athena fallback swap remains active");
    }

    private Result writePrivileged(Path path, String value) {
        CommandResult result = commandWithInput(value + "\n", "sudo", "-n", "/usr/bin/tee", path.toString());
        if (!result.success) return Result.failure(result.detail);
        String actual = readFirst(path).trim();
        return actual.equals(value)
                ? Result.success(path + "=" + value + " verified")
                : Result.failure("readback for " + path + " was '" + actual + "'");
    }

    private void cleanupFailedLoop(String loop) {
        command("sudo", "-n", "/sbin/losetup", "-d", loop);
        cachedLoopDevices = List.of();
        deleteSwapFileQuietly();
    }

    private void deleteSwapFileQuietly() {
        try {
            Files.deleteIfExists(SWAP_FILE);
        } catch (IOException ignored) {
            // The primary operation reports the actionable failure.
        }
    }

    private Result writePrivilegedNoReadback(Path path, String value) {
        CommandResult result = commandWithInput(value + "\n", "sudo", "-n", "/usr/bin/tee", path.toString());
        return result.success
                ? Result.success("Wrote " + path)
                : Result.failure(result.detail);
    }

    private boolean waitForWebServer(boolean running) {
        for (int attempt = 0; attempt < 10; attempt++) {
            if (niWebServerRunning() == running) return true;
            try {
                Thread.sleep(100);
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        return niWebServerRunning() == running;
    }

    private List<String> activeSwaps() {
        List<String> lines = readLines(Path.of("/proc/swaps"));
        return lines.size() <= 1 ? List.of() : lines.subList(1, lines.size());
    }

    private List<String> attachedLoopDevices() {
        List<String> cached = cachedLoopDevices;
        if (cached != null) return cached;
        CommandResult result = command("/sbin/losetup", "-j", SWAP_FILE.toString());
        if (!result.success || result.detail.isBlank()) {
            cachedLoopDevices = List.of();
            return cachedLoopDevices;
        }
        List<String> devices = new ArrayList<>();
        result.detail.lines().forEach(line -> {
            int colon = line.indexOf(':');
            if (colon > 0) devices.add(line.substring(0, colon));
        });
        cachedLoopDevices = List.copyOf(devices);
        return cachedLoopDevices;
    }

    private static String firstField(String line) {
        int whitespace = line.indexOf('\t');
        if (whitespace < 0) whitespace = line.indexOf(' ');
        return whitespace < 0 ? line.trim() : line.substring(0, whitespace).trim();
    }

    private static boolean allowedSysctl(String key) {
        return key.equals("vm.overcommit_memory") || key.equals("vm.swappiness");
    }

    private static CommandResult command(String... arguments) {
        return commandWithInput(null, arguments);
    }

    private static CommandResult commandWithInput(String input, String... arguments) {
        Process process = null;
        try {
            process = new ProcessBuilder(arguments).redirectErrorStream(true).start();
            if (input != null) {
                try (OutputStream output = process.getOutputStream()) {
                    output.write(input.getBytes(StandardCharsets.UTF_8));
                }
            }
            boolean complete = process.waitFor(COMMAND_TIMEOUT.toMillis(), TimeUnit.MILLISECONDS);
            if (!complete) {
                process.destroyForcibly();
                return new CommandResult(false, "timed out");
            }
            String output = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
            return new CommandResult(
                    process.exitValue() == 0,
                    process.exitValue() == 0
                            ? output
                            : "exit " + process.exitValue() + (output.isBlank() ? "" : ": " + output));
        } catch (IOException exception) {
            return new CommandResult(false, exception.getMessage());
        } catch (InterruptedException exception) {
            if (process != null) process.destroyForcibly();
            Thread.currentThread().interrupt();
            return new CommandResult(false, "interrupted");
        }
    }

    private static String readFirst(Path... paths) {
        for (Path path : paths) {
            try {
                if (Files.isReadable(path)) return Files.readString(path, StandardCharsets.UTF_8);
            } catch (IOException ignored) {
                // Try the next known kernel path.
            }
        }
        return "";
    }

    private static List<String> readLines(Path path) {
        try {
            return Files.isReadable(path) ? Files.readAllLines(path, StandardCharsets.UTF_8) : List.of();
        } catch (IOException ignored) {
            return new ArrayList<>();
        }
    }

    private static long kib(String line) {
        String[] fields = line.trim().split("\\s+");
        if (fields.length < 2) return -1;
        try {
            return Long.parseLong(fields[1]) * 1024L;
        } catch (NumberFormatException ignored) {
            return -1;
        }
    }

    private record CommandResult(boolean success, String detail) { }
}
