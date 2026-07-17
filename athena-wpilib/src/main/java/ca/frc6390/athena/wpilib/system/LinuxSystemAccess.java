package ca.frc6390.athena.wpilib.system;

import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
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
    private static final Duration COMMAND_TIMEOUT = Duration.ofSeconds(3);
    private final boolean realRobot;

    LinuxSystemAccess(boolean realRobot) {
        this.realRobot = realRobot;
    }

    @Override
    public boolean realRobot() {
        return realRobot;
    }

    @Override
    public boolean linux() {
        return System.getProperty("os.name", "").toLowerCase(Locale.ROOT).contains("linux");
    }

    @Override
    public String target() {
        String model = readFirst(
                Path.of("/proc/device-tree/model"),
                Path.of("/sys/firmware/devicetree/base/model"));
        if (!model.isBlank()) return model.replace("\0", "").trim();
        return realRobot ? "Unknown Linux robot" : "Simulation";
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
    public boolean hasActiveZram() {
        for (String line : readLines(Path.of("/proc/swaps"))) {
            if (line.toLowerCase(Locale.ROOT).contains("zram")) return true;
        }
        return false;
    }

    @Override
    public Result stopNiWebServer() {
        Result server = command("sudo", "-n", "/usr/bin/pkill", "-TERM", "-f",
                "^/usr/local/natinst/share/NIWebServer/SystemWebServer");
        Result container = command("sudo", "-n", "/usr/bin/pkill", "-TERM", "-f",
                "^/usr/local/natinst/share/NIWebServer/NIWebServiceContainer");
        if (server.success() || container.success()) return Result.success("NI web services stopped");
        // pkill returns 1 when no matching service exists; that is already the desired state.
        if (server.detail().contains("exit 1") && container.detail().contains("exit 1")) {
            return Result.success("NI web services already stopped");
        }
        return Result.failure("NI web service: " + server.detail() + "; " + container.detail());
    }

    @Override
    public Result setSysctl(String key, int value) {
        if (!key.equals("vm.overcommit_memory") && !key.equals("vm.swappiness")) {
            return Result.failure("Unsupported sysctl " + key);
        }
        Result result = command("sudo", "-n", "/sbin/sysctl", "-w", key + "=" + value);
        if (!result.success()) return result;
        String actual = readFirst(Path.of("/proc/sys/" + key.replace('.', '/'))).trim();
        return Integer.toString(value).equals(actual)
                ? Result.success(key + "=" + value)
                : Result.failure("readback was '" + actual + "'");
    }

    @Override
    public Result enableSwapFile(long bytes) {
        try {
            Files.createDirectories(SWAP_FILE.getParent());
            if (!Files.exists(SWAP_FILE) || Files.size(SWAP_FILE) != bytes) {
                try (RandomAccessFile file = new RandomAccessFile(SWAP_FILE.toFile(), "rw")) {
                    file.setLength(bytes);
                }
                try {
                    Files.setPosixFilePermissions(SWAP_FILE, PosixFilePermissions.fromString("rw-------"));
                } catch (UnsupportedOperationException ignored) {
                    // roboRIO is POSIX; this only protects non-POSIX test hosts.
                }
            }
        } catch (IOException exception) {
            return Result.failure("Cannot create swap file: " + exception.getMessage());
        }
        Result format = command("sudo", "-n", "/sbin/mkswap", SWAP_FILE.toString());
        if (!format.success()) return Result.failure("mkswap: " + format.detail());
        Result enable = command("sudo", "-n", "/sbin/swapon", SWAP_FILE.toString());
        if (!enable.success()) return Result.failure("swapon: " + enable.detail());
        boolean active = readLines(Path.of("/proc/swaps")).stream()
                .anyMatch(line -> line.startsWith(SWAP_FILE.toString()));
        return active
                ? Result.success("Enabled " + bytes / (1024 * 1024) + " MiB fallback swap")
                : Result.failure("swapon succeeded but the swap file is absent from /proc/swaps");
    }

    private static Result command(String... arguments) {
        try {
            Process process = new ProcessBuilder(arguments).redirectErrorStream(true).start();
            boolean complete = process.waitFor(COMMAND_TIMEOUT.toMillis(), TimeUnit.MILLISECONDS);
            if (!complete) {
                process.destroyForcibly();
                return Result.failure("timed out");
            }
            String output = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
            return process.exitValue() == 0
                    ? Result.success(output.isBlank() ? String.join(" ", arguments) : output)
                    : Result.failure("exit " + process.exitValue() + (output.isBlank() ? "" : ": " + output));
        } catch (IOException exception) {
            return Result.failure(exception.getMessage());
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            return Result.failure("interrupted");
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
}
