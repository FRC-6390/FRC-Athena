package ca.frc6390.athena.benchmarks.rebuilt;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.lang.management.ManagementFactory;
import java.util.Arrays;
import java.util.Locale;

/**
 * Runs the Rebuilt-shaped robot through WPILib's real TimedRobot loop body and reports the cost of one 20 ms
 * cycle: main-thread CPU time, wall time, main-thread allocation, and whole-process CPU time (which includes
 * background threads such as NetworkTables, ARCP and replay flushing).
 *
 * <p>The FPGA clock is paused and stepped by 20 ms per cycle, so everything rate-limited on robot time runs at
 * the same per-cycle rate it would on the robot. Simulated physics still runs here (the roboRIO runs vendor
 * hardware instead), so treat the absolute numbers as desktop-relative and compare before/after runs.
 *
 * <p>Usage: {@code ./gradlew :athena-benchmarks:robotLoopProfile [-Pcycles=20000] [-Pjfr=/path/out.jfr]}
 */
public final class RobotLoopProfile {
    private static final double PERIOD_SECONDS = 0.02;

    private RobotLoopProfile() {
    }

    public static void main(String[] args) {
        int cycles = args.length > 0 ? Integer.parseInt(args[0]) : 20_000;
        int warmup = Math.max(2_000, cycles / 4);
        if (!HAL.initialize(500, 0)) throw new IllegalStateException("HAL failed to initialize");
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        RebuiltRobot robot = new RebuiltRobot();
        robot.robotInit();
        robot.simulationInit();
        for (int i = 0; i < 50; i++) step(robot, i, false);

        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < warmup; i++) step(robot, i, true);

        com.sun.management.ThreadMXBean threads =
                (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
        com.sun.management.OperatingSystemMXBean os =
                (com.sun.management.OperatingSystemMXBean) ManagementFactory.getOperatingSystemMXBean();
        long[] wall = new long[cycles];
        long cpuStart = threads.getCurrentThreadCpuTime();
        long processCpuStart = os.getProcessCpuTime();
        long allocatedStart = threads.getCurrentThreadAllocatedBytes();
        long gcStart = gcCount();
        for (int i = 0; i < cycles; i++) {
            long start = System.nanoTime();
            step(robot, warmup + i, true);
            wall[i] = System.nanoTime() - start;
        }
        long cpu = threads.getCurrentThreadCpuTime() - cpuStart;
        long processCpu = os.getProcessCpuTime() - processCpuStart;
        long allocated = threads.getCurrentThreadAllocatedBytes() - allocatedStart;
        long gcs = gcCount() - gcStart;
        Arrays.sort(wall);

        double simulatedSeconds = cycles * PERIOD_SECONDS;
        System.out.println();
        System.out.println("Athena robot loop profile (" + cycles + " measured cycles after " + warmup + " warmup)");
        System.out.printf(Locale.ROOT, "  main-thread CPU      %10.1f us/cycle%n", cpu / 1_000.0 / cycles);
        System.out.printf(Locale.ROOT, "  wall p50 / p99 / max %10.1f / %.1f / %.1f us%n",
                wall[cycles / 2] / 1_000.0, wall[(int) (cycles * 0.99)] / 1_000.0, wall[cycles - 1] / 1_000.0);
        System.out.printf(Locale.ROOT, "  main-thread alloc    %10.0f bytes/cycle%n", (double) allocated / cycles);
        System.out.printf(Locale.ROOT, "  process CPU          %10.1f us/cycle (%.1f%% of one core at 50 Hz)%n",
                processCpu / 1_000.0 / cycles, 100.0 * processCpu / 1.0e9 / simulatedSeconds);
        System.out.printf(Locale.ROOT, "  GC collections       %10d%n", gcs);
        robot.close();
        System.exit(0);
    }

    /** One robot cycle: a driver station packet with sticks and triggers, then the TimedRobot loop body. */
    private static void step(RebuiltRobot robot, int cycle, boolean enabled) {
        double phase = (cycle % 500) / 500.0;
        // Driver: drive in a slow circle, hold the shoot trigger for half of every 10 s.
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickButtonCount(0, 10);
        DriverStationSim.setJoystickPOVCount(0, 1);
        DriverStationSim.setJoystickAxis(0, 0, 0.6 * Math.sin(phase * 2.0 * Math.PI));
        DriverStationSim.setJoystickAxis(0, 1, 0.6 * Math.cos(phase * 2.0 * Math.PI));
        DriverStationSim.setJoystickAxis(0, 4, 0.2);
        DriverStationSim.setJoystickAxis(0, 3, (cycle / 250) % 2 == 0 ? 1.0 : 0.0);
        DriverStationSim.setJoystickAxisCount(1, 6);
        DriverStationSim.setJoystickButtonCount(1, 10);
        DriverStationSim.setJoystickPOVCount(1, 1);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        robot.cycle();
        SimHooks.stepTimingAsync(PERIOD_SECONDS);
    }

    private static long gcCount() {
        long total = 0;
        for (var gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            total += Math.max(0, gc.getCollectionCount());
        }
        return total;
    }
}
