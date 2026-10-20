package ca.frc6390.athena.benchmarks.rebuilt;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.lang.management.ClassLoadingMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.Arrays;
import java.util.Locale;

/**
 * Measures what the Rebuilt-shaped robot costs to bring up: {@code new RebuiltRobot()}, {@code robotInit()},
 * {@code simulationInit()}, then the first 250 loop cycles (5 s after power-up, disabled) against steady state
 * (enabled). Each phase reports wall time, main-thread CPU, whole-process CPU, main-thread allocation, classes
 * loaded and heap growth. Startup cycles run disabled, as they do on a real robot after power-up.
 *
 * <p>Usage: {@code ./gradlew :athena-benchmarks:robotStartupProfile [-Pcycles=2000] [-Pjfr=/path/startup.jfr]}
 *
 * <p>Desktop numbers are relative only (the simulated backends skip CAN and vendor JNI); compare before/after.
 */
public final class RobotStartupProfile {
    private static final int STARTUP_CYCLES = 250;
    private static final double PERIOD_SECONDS = 0.02;

    private RobotStartupProfile() {
    }

    /** Snapshot of the counters a phase is measured against. */
    private record Mark(long wall, long mainCpu, long process, long allocated, long classes, long heap) {
    }

    private static final com.sun.management.ThreadMXBean THREADS =
            (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
    private static final com.sun.management.OperatingSystemMXBean OS =
            (com.sun.management.OperatingSystemMXBean) ManagementFactory.getOperatingSystemMXBean();
    private static final ClassLoadingMXBean CLASSES = ManagementFactory.getClassLoadingMXBean();
    private static final MemoryMXBean MEMORY = ManagementFactory.getMemoryMXBean();

    private static Mark mark() {
        return new Mark(System.nanoTime(), THREADS.getCurrentThreadCpuTime(), OS.getProcessCpuTime(),
                THREADS.getCurrentThreadAllocatedBytes(), CLASSES.getTotalLoadedClassCount(),
                MEMORY.getHeapMemoryUsage().getUsed());
    }

    private static void report(String phase, Mark start) {
        Mark end = mark();
        System.out.printf(Locale.ROOT,
                "  %-18s wall %9.1f ms | main CPU %9.1f ms | process CPU %9.1f ms | alloc %8.1f MB | "
                        + "classes +%5d | heap %+7.1f MB%n",
                phase,
                (end.wall() - start.wall()) / 1.0e6,
                (end.mainCpu() - start.mainCpu()) / 1.0e6,
                (end.process() - start.process()) / 1.0e6,
                (end.allocated() - start.allocated()) / 1.0e6,
                end.classes() - start.classes(),
                (end.heap() - start.heap()) / 1.0e6);
    }

    public static void main(String[] args) {
        int steadyCycles = args.length > 0 ? Integer.parseInt(args[0]) : 2_000;
        if (!HAL.initialize(500, 0)) throw new IllegalStateException("HAL failed to initialize");
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        System.out.println();
        System.out.println("Athena robot startup profile (" + STARTUP_CYCLES + " startup cycles, "
                + steadyCycles + " steady cycles)");

        Mark start = mark();
        RebuiltRobot robot = new RebuiltRobot();
        report("construct", start);

        start = mark();
        robot.robotInit();
        report("robotInit", start);

        start = mark();
        robot.simulationInit();
        report("simulationInit", start);

        // First 5 s after boot: disabled, every cycle timed individually on the main thread.
        long[] startupCpu = new long[STARTUP_CYCLES];
        long startupWallStart = System.nanoTime();
        long startupStart = THREADS.getCurrentThreadCpuTime();
        for (int i = 0; i < STARTUP_CYCLES; i++) {
            if (i == 0 || i == 12 || i == STARTUP_CYCLES) {
                // Wall-clock markers so a JFR recording can be cut to the same window (see the startup notes).
                System.out.println("marker cycle " + i + " epoch_ms " + System.currentTimeMillis());
            }
            long cpuBefore = THREADS.getCurrentThreadCpuTime();
            RobotLoopProfile.step(robot, i, false);
            startupCpu[i] = THREADS.getCurrentThreadCpuTime() - cpuBefore;
        }
        long startupWall = System.nanoTime() - startupWallStart;
        long startupMainCpu = THREADS.getCurrentThreadCpuTime() - startupStart;

        // Disabled steady state (cycles 250-500): the like-for-like baseline for the startup window, which is disabled too.
        long[] disabledCpu = new long[STARTUP_CYCLES];
        long disabledStart = THREADS.getCurrentThreadCpuTime();
        for (int i = 0; i < STARTUP_CYCLES; i++) {
            long cpuBefore = THREADS.getCurrentThreadCpuTime();
            RobotLoopProfile.step(robot, STARTUP_CYCLES + i, false);
            disabledCpu[i] = THREADS.getCurrentThreadCpuTime() - cpuBefore;
        }
        long disabledMainCpu = THREADS.getCurrentThreadCpuTime() - disabledStart;

        // Steady state: enabled, after a warmup, same loop body.
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        int warmup = Math.max(2_000, steadyCycles / 4);
        for (int i = 0; i < warmup; i++) RobotLoopProfile.step(robot, 2 * STARTUP_CYCLES + i, true);
        long[] steadyCpu = new long[steadyCycles];
        long steadyStart = THREADS.getCurrentThreadCpuTime();
        for (int i = 0; i < steadyCycles; i++) {
            long cpuBefore = THREADS.getCurrentThreadCpuTime();
            RobotLoopProfile.step(robot, 2 * STARTUP_CYCLES + warmup + i, true);
            steadyCpu[i] = THREADS.getCurrentThreadCpuTime() - cpuBefore;
        }
        long steadyMainCpu = THREADS.getCurrentThreadCpuTime() - steadyStart;

        System.out.println();
        System.out.printf(Locale.ROOT, "  %-18s %s%n", "loop cycles", "main-thread CPU per cycle (us)");
        printCycleStats("  first 250 (0-5 s)", startupCpu, startupMainCpu, startupWall);
        printCycleStats("  next 250 (5-10 s, disabled)", disabledCpu, disabledMainCpu, -1);
        printCycleStats("  steady (enabled)", steadyCpu, steadyMainCpu, -1);
        printTimeline(startupCpu);
        System.out.println();
        robot.close();
        System.exit(0);
    }

    /** The first cycles and the most expensive startup cycles, to show whether cost is a burst or spread out. */
    private static void printTimeline(long[] cpuNanos) {
        StringBuilder first = new StringBuilder("  first cycles (us):");
        for (int i = 0; i < Math.min(12, cpuNanos.length); i++) {
            first.append(String.format(Locale.ROOT, " %d:%.0f", i, cpuNanos[i] / 1_000.0));
        }
        System.out.println(first);
        Integer[] order = new Integer[cpuNanos.length];
        for (int i = 0; i < order.length; i++) order[i] = i;
        Arrays.sort(order, (a, b) -> Long.compare(cpuNanos[b], cpuNanos[a]));
        StringBuilder worst = new StringBuilder("  costliest cycles (cycle:us):");
        for (int i = 0; i < Math.min(8, order.length); i++) {
            worst.append(String.format(Locale.ROOT, " %d:%.0f", order[i], cpuNanos[order[i]] / 1_000.0));
        }
        System.out.println(worst);
    }

    private static void printCycleStats(String label, long[] cpuNanos, long totalCpu, long totalWall) {
        long[] sorted = cpuNanos.clone();
        Arrays.sort(sorted);
        int n = sorted.length;
        System.out.printf(Locale.ROOT,
                "%-20s mean %8.1f | p50 %8.1f | p99 %8.1f | max %9.1f | total CPU %8.1f ms%s%n",
                label,
                totalCpu / 1_000.0 / n,
                sorted[n / 2] / 1_000.0,
                sorted[(int) (n * 0.99)] / 1_000.0,
                sorted[n - 1] / 1_000.0,
                totalCpu / 1.0e6,
                totalWall < 0 ? "" : String.format(Locale.ROOT, " | wall %8.1f ms", totalWall / 1.0e6));
    }
}
