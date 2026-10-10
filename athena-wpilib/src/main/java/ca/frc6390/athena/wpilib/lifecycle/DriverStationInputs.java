package ca.frc6390.athena.wpilib.lifecycle;

import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

/** Samples WPILib's driver station, joysticks included, into a {@link RuntimeInputSnapshot}. */
final class DriverStationInputs {
    private DriverStationInputs() {
    }

    /** Reads every joystick port and the driver station state. Called once per cycle when replay is enabled. */
    static RuntimeInputSnapshot sample() {
        List<RuntimeInputSnapshot.Joystick> joysticks = new ArrayList<>(RuntimeInputSnapshot.JOYSTICK_PORTS);
        for (int port = 0; port < RuntimeInputSnapshot.JOYSTICK_PORTS; port++) {
            joysticks.add(joystick(port));
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
        return new RuntimeInputSnapshot(joysticks, state);
    }

    private static RuntimeInputSnapshot.Joystick joystick(int port) {
        if (!DriverStation.isJoystickConnected(port)) {
            return new RuntimeInputSnapshot.Joystick(port, false, "", 0, new double[0], 0L, new int[0]);
        }
        int axisCount = Math.min(DriverStation.getStickAxisCount(port), RuntimeInputSnapshot.AXES);
        double[] axes = new double[axisCount];
        for (int axis = 0; axis < axisCount; axis++) {
            axes[axis] = DriverStation.getStickAxis(port, axis);
        }
        int povCount = Math.min(DriverStation.getStickPOVCount(port), RuntimeInputSnapshot.POVS);
        int[] povs = new int[povCount];
        for (int pov = 0; pov < povCount; pov++) {
            povs[pov] = DriverStation.getStickPOV(port, pov);
        }
        return new RuntimeInputSnapshot.Joystick(
                port,
                true,
                DriverStation.getJoystickName(port),
                DriverStation.getJoystickType(port),
                axes,
                Integer.toUnsignedLong(DriverStation.getStickButtons(port)),
                povs);
    }
}
