package ca.frc6390.athena.robot;

import java.util.ArrayList;
import java.util.List;

/**
 * Driver station and joystick state read at the start of one runtime cycle. Controllers read the same values
 * during the cycle, so recording this snapshot is enough to re-drive bindings in replay.
 *
 * @param joysticks joystick states for the ports that were read; ports not listed are disconnected
 * @param driverStation driver station state
 */
public record RuntimeInputSnapshot(List<Joystick> joysticks, DriverStationState driverStation) {
    /** Number of joystick ports the driver station exposes. */
    public static final int JOYSTICK_PORTS = 6;
    /** Axes recorded per joystick. */
    public static final int AXES = 6;
    /** POV values recorded per joystick. */
    public static final int POVS = 4;
    private static final RuntimeInputSnapshot EMPTY =
            new RuntimeInputSnapshot(List.of(), DriverStationState.DISCONNECTED);

    public RuntimeInputSnapshot {
        joysticks = joysticks == null ? List.of() : List.copyOf(joysticks);
        driverStation = driverStation == null ? DriverStationState.DISCONNECTED : driverStation;
    }

    /** Returns an empty snapshot: no joysticks, default driver station state. */
    public static RuntimeInputSnapshot empty() {
        return EMPTY;
    }

    /**
     * Returns the state of one joystick port.
     *
     * @param port zero-based port
     * @return joystick state; a disconnected placeholder when the port was not read
     */
    public Joystick joystick(int port) {
        for (Joystick joystick : joysticks) {
            if (joystick.port() == port) {
                return joystick;
            }
        }
        return Joystick.disconnected(port);
    }

    /**
     * One joystick port.
     *
     * @param port zero-based port
     * @param connected true when the driver station sees the device
     * @param name device name, empty when disconnected
     * @param type WPILib joystick type
     * @param axes axis values, up to {@link #AXES}
     * @param buttons button bitmask; bit {@code n} is WPILib button {@code n + 1}
     * @param povs POV angles in degrees (-1 when released), up to {@link #POVS}
     */
    public record Joystick(
            int port,
            boolean connected,
            String name,
            int type,
            double[] axes,
            long buttons,
            int[] povs) {
        public Joystick {
            name = name == null ? "" : name;
            axes = axes == null ? new double[0] : axes.clone();
            povs = povs == null ? new int[0] : povs.clone();
        }

        private static final Joystick[] DISCONNECTED = disconnectedPorts();

        /** Returns the disconnected placeholder for a port; instances for the standard ports are shared. */
        static Joystick disconnected(int port) {
            if (port >= 0 && port < DISCONNECTED.length) {
                return DISCONNECTED[port];
            }
            return new Joystick(port, false, "", 0, new double[0], 0L, new int[0]);
        }

        private static Joystick[] disconnectedPorts() {
            Joystick[] ports = new Joystick[JOYSTICK_PORTS];
            for (int port = 0; port < ports.length; port++) {
                ports[port] = new Joystick(port, false, "", 0, new double[0], 0L, new int[0]);
            }
            return ports;
        }

        /** Returns an axis value, or 0 when the device does not report that axis. */
        public double axis(int axis) {
            return axis >= 0 && axis < axes.length ? axes[axis] : 0.0;
        }

        /** Returns a WPILib one-based button state. Disconnected devices read as released. */
        public boolean button(int button) {
            return connected && button >= 1 && button <= 64 && ((buttons >>> (button - 1)) & 1L) != 0;
        }

        /** Returns POV angle {@code index}, or -1 when released or not reported. */
        public int pov(int index) {
            return index >= 0 && index < povs.length ? povs[index] : -1;
        }

        @Override
        public double[] axes() {
            return axes.clone();
        }

        @Override
        public int[] povs() {
            return povs.clone();
        }
    }

    /**
     * Driver station state.
     *
     * @param enabled robot enabled
     * @param autonomous autonomous mode selected
     * @param test test mode selected
     * @param fmsAttached FMS connected
     * @param dsAttached driver station connected
     * @param alliance alliance color, "red", "blue", or empty when unknown
     * @param matchTime seconds remaining in the match, or -1 when unknown
     * @param eventName event name, empty when unknown
     * @param matchType match type name, such as NONE, PRACTICE, QUALIFICATION, ELIMINATION
     * @param matchNumber match number
     * @param gameMessage game-specific message
     */
    public record DriverStationState(
            boolean enabled,
            boolean autonomous,
            boolean test,
            boolean fmsAttached,
            boolean dsAttached,
            String alliance,
            double matchTime,
            String eventName,
            String matchType,
            int matchNumber,
            String gameMessage) {
        public static final DriverStationState DISCONNECTED = new DriverStationState(
                false, false, false, false, false, "", -1.0, "", "NONE", 0, "");

        public DriverStationState {
            alliance = alliance == null ? "" : alliance;
            eventName = eventName == null ? "" : eventName;
            matchType = matchType == null ? "NONE" : matchType;
            gameMessage = gameMessage == null ? "" : gameMessage;
        }
    }
}
