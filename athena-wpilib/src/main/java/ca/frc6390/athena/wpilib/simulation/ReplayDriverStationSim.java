package ca.frc6390.athena.wpilib.simulation;

import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.Locale;
import java.util.function.Consumer;

/**
 * Drives WPILib's simulated driver station from recorded input, so robot code that calls {@link DriverStation}
 * statics directly (for example {@code DriverStation.isTeleopEnabled()}) sees the recorded state during replay.
 *
 * <p>Use with {@code ReplaySimulation.replay(file).driverStation(ReplayDriverStationSim.sink())}. Each frame sets the
 * control word, alliance station, match information, and joystick state of every port, then calls
 * {@link DriverStationSim#notifyNewData()}. The injected {@code RuntimeInputs} state is still set as before.</p>
 *
 * <p>The state stays set after the replay. Call {@link #reset()} when the replay is finished, or the next robot code
 * in the same process reads the replayed driver station.</p>
 */
public final class ReplayDriverStationSim {
    private ReplayDriverStationSim() {
    }

    /** Returns a sink that applies each recorded frame to the simulated driver station. */
    public static Consumer<RuntimeInputSnapshot> sink() {
        return ReplayDriverStationSim::apply;
    }

    /** Applies one recorded frame to the simulated driver station and notifies WPILib of the new data. */
    public static void apply(RuntimeInputSnapshot input) {
        RuntimeInputSnapshot.DriverStationState state = input.driverStation();
        DriverStationSim.setDsAttached(state.dsAttached());
        DriverStationSim.setFmsAttached(state.fmsAttached());
        DriverStationSim.setAllianceStationId(allianceStation(state.alliance()));
        DriverStationSim.setTest(state.test());
        DriverStationSim.setAutonomous(state.autonomous());
        DriverStationSim.setEnabled(state.enabled());
        DriverStationSim.setMatchTime(state.matchTime());
        DriverStationSim.setEventName(state.eventName());
        DriverStationSim.setMatchType(matchType(state.matchType()));
        DriverStationSim.setMatchNumber(state.matchNumber());
        DriverStationSim.setGameSpecificMessage(state.gameMessage());
        for (int port = 0; port < RuntimeInputSnapshot.JOYSTICK_PORTS; port++) {
            applyJoystick(port, input.joystick(port));
        }
        DriverStationSim.notifyNewData();
    }

    /** Clears the simulated driver station state that {@link #apply} set. */
    public static void reset() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    private static void applyJoystick(int port, RuntimeInputSnapshot.Joystick joystick) {
        if (!joystick.connected()) {
            // WPILib treats a stick with no axes, buttons or POVs as disconnected.
            DriverStationSim.setJoystickAxisCount(port, 0);
            DriverStationSim.setJoystickButtonCount(port, 0);
            DriverStationSim.setJoystickPOVCount(port, 0);
            return;
        }
        DriverStationSim.setJoystickName(port, joystick.name());
        DriverStationSim.setJoystickType(port, joystick.type());
        double[] axes = joystick.axes();
        DriverStationSim.setJoystickAxisCount(port, axes.length);
        for (int axis = 0; axis < axes.length; axis++) {
            DriverStationSim.setJoystickAxis(port, axis, axes[axis]);
        }
        DriverStationSim.setJoystickButtonCount(port, 64);
        // Buttons go one at a time: WPILib button n+1 is bit n of the recorded mask.
        // The HAL carries 32 buttons per stick, the same width the live sampler reads, so the mask is lossless.
        DriverStationSim.setJoystickButtons(port, (int) joystick.buttons());
        int[] povs = joystick.povs();
        DriverStationSim.setJoystickPOVCount(port, povs.length);
        for (int pov = 0; pov < povs.length; pov++) {
            DriverStationSim.setJoystickPOV(port, pov, povs[pov]);
        }
    }

    private static AllianceStationID allianceStation(String alliance) {
        return switch (alliance.toLowerCase(Locale.ROOT)) {
            case "red" -> AllianceStationID.Red1;
            case "blue" -> AllianceStationID.Blue1;
            default -> AllianceStationID.Unknown;
        };
    }

    /** Matches the recorded name ({@code Qualification}, {@code QUALIFICATION}, ...) case-insensitively. */
    private static DriverStation.MatchType matchType(String name) {
        for (DriverStation.MatchType type : DriverStation.MatchType.values()) {
            if (type.name().equalsIgnoreCase(name)) {
                return type;
            }
        }
        return DriverStation.MatchType.None;
    }
}
