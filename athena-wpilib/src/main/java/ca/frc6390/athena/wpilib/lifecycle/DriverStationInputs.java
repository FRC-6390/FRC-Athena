package ca.frc6390.athena.wpilib.lifecycle;

import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/** Samples WPILib's driver station, joysticks included, into a {@link RuntimeInputSnapshot}. */
final class DriverStationInputs {
    // The previous sample. Snapshots are immutable, so parts whose values did not change are handed out again
    // instead of being rebuilt every cycle. Guarded by the class monitor.
    private static final RuntimeInputSnapshot.Joystick[] LAST_JOYSTICKS =
            new RuntimeInputSnapshot.Joystick[RuntimeInputSnapshot.JOYSTICK_PORTS];
    private static RuntimeInputSnapshot.DriverStationState lastState;
    private static RuntimeInputSnapshot lastSnapshot;
    private static final int[] LAST_AXIS_COUNTS = new int[RuntimeInputSnapshot.JOYSTICK_PORTS];
    private static final int[] LAST_POV_COUNTS = new int[RuntimeInputSnapshot.JOYSTICK_PORTS];
    private static final double[] AXES = new double[RuntimeInputSnapshot.AXES];
    private static final int[] POVS = new int[RuntimeInputSnapshot.POVS];

    private DriverStationInputs() {
    }

    /** Reads every joystick port and the driver station state. Called once per cycle when replay is enabled. */
    static synchronized RuntimeInputSnapshot sample() {
        boolean unchanged = lastSnapshot != null;
        for (int port = 0; port < RuntimeInputSnapshot.JOYSTICK_PORTS; port++) {
            RuntimeInputSnapshot.Joystick joystick = joystick(port, LAST_JOYSTICKS[port]);
            unchanged &= joystick == LAST_JOYSTICKS[port];
            LAST_JOYSTICKS[port] = joystick;
        }
        var alliance = DriverStation.getAlliance();
        RuntimeInputSnapshot.DriverStationState state = new RuntimeInputSnapshot.DriverStationState(
                DriverStation.isEnabled(),
                DriverStation.isAutonomous(),
                DriverStation.isTest(),
                DriverStation.isFMSAttached(),
                DriverStation.isDSAttached(),
                alliance.isPresent() ? alliance.get().name().toLowerCase(java.util.Locale.ROOT) : "",
                DriverStation.getMatchTime(),
                DriverStation.getEventName(),
                DriverStation.getMatchType().name(),
                DriverStation.getMatchNumber(),
                DriverStation.getGameSpecificMessage());
        if (state.equals(lastState)) {
            state = lastState;
        } else {
            unchanged = false;
            lastState = state;
        }
        if (!unchanged) {
            lastSnapshot = new RuntimeInputSnapshot(List.of(LAST_JOYSTICKS), state);
        }
        return lastSnapshot;
    }

    /** Reads one port; returns {@code previous} when every value matches it. */
    private static RuntimeInputSnapshot.Joystick joystick(int port, RuntimeInputSnapshot.Joystick previous) {
        if (!DriverStation.isJoystickConnected(port)) {
            if (previous != null && !previous.connected()) {
                return previous;
            }
            return new RuntimeInputSnapshot.Joystick(port, false, "", 0, new double[0], 0L, new int[0]);
        }
        int axisCount = Math.min(DriverStation.getStickAxisCount(port), RuntimeInputSnapshot.AXES);
        for (int axis = 0; axis < axisCount; axis++) {
            AXES[axis] = DriverStation.getStickAxis(port, axis);
        }
        int povCount = Math.min(DriverStation.getStickPOVCount(port), RuntimeInputSnapshot.POVS);
        for (int pov = 0; pov < povCount; pov++) {
            POVS[pov] = DriverStation.getStickPOV(port, pov);
        }
        String name = DriverStation.getJoystickName(port);
        int type = DriverStation.getJoystickType(port);
        long buttons = Integer.toUnsignedLong(DriverStation.getStickButtons(port));
        if (previous != null
                && previous.connected()
                && previous.type() == type
                && previous.buttons() == buttons
                && Objects.equals(previous.name(), name == null ? "" : name)
                && LAST_AXIS_COUNTS[port] == axisCount
                && LAST_POV_COUNTS[port] == povCount
                && sameAxes(previous, axisCount)
                && samePovs(previous, povCount)) {
            return previous;
        }
        LAST_AXIS_COUNTS[port] = axisCount;
        LAST_POV_COUNTS[port] = povCount;
        return new RuntimeInputSnapshot.Joystick(
                port,
                true,
                name,
                type,
                Arrays.copyOf(AXES, axisCount),
                buttons,
                Arrays.copyOf(POVS, povCount));
    }

    private static boolean sameAxes(RuntimeInputSnapshot.Joystick previous, int axisCount) {
        // axis() reads the record's own array without the copy that axes() makes.
        for (int axis = 0; axis < axisCount; axis++) {
            if (Double.doubleToRawLongBits(previous.axis(axis)) != Double.doubleToRawLongBits(AXES[axis])) {
                return false;
            }
        }
        return true;
    }

    private static boolean samePovs(RuntimeInputSnapshot.Joystick previous, int povCount) {
        for (int pov = 0; pov < povCount; pov++) {
            if (previous.pov(pov) != POVS[pov]) {
                return false;
            }
        }
        return true;
    }
}
