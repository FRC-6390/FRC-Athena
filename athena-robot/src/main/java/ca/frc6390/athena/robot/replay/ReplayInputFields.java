package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.runtime.wire.Wire;
import java.util.ArrayList;
import java.util.List;

/**
 * Signal layout for driver station input, joystick input and localization summaries, plus the
 * decoding of those signals from a replay cursor.
 *
 * <p>Paths: {@code input/driverStation/<field>}, {@code input/joystick/<port>/<field>} for ports 0 to 5, and
 * {@code localization/<index>/<field>}.</p>
 */
final class ReplayInputFields {
    static final String DRIVER_STATION = "input/driverStation/";
    static final String JOYSTICK = "input/joystick/";
    static final String LOCALIZATION = "localization/";

    static final ReplayEncoder.Field[] DRIVER_STATION_FIELDS = {
            new ReplayEncoder.Field("enabled", Wire.TYPE_BOOL, ""),
            new ReplayEncoder.Field("autonomous", Wire.TYPE_BOOL, ""),
            new ReplayEncoder.Field("test", Wire.TYPE_BOOL, ""),
            new ReplayEncoder.Field("fmsAttached", Wire.TYPE_BOOL, ""),
            new ReplayEncoder.Field("dsAttached", Wire.TYPE_BOOL, ""),
            new ReplayEncoder.Field("alliance", Wire.TYPE_STRING, ""),
            new ReplayEncoder.Field("matchTime", Wire.TYPE_F64, "s"),
            new ReplayEncoder.Field("eventName", Wire.TYPE_STRING, ""),
            new ReplayEncoder.Field("matchType", Wire.TYPE_STRING, ""),
            new ReplayEncoder.Field("matchNumber", Wire.TYPE_I64, ""),
            new ReplayEncoder.Field("gameMessage", Wire.TYPE_STRING, ""),
    };
    static final int DS_ENABLED = 0;
    static final int DS_AUTONOMOUS = 1;
    static final int DS_TEST = 2;
    static final int DS_FMS = 3;
    static final int DS_DS_ATTACHED = 4;
    static final int DS_ALLIANCE = 5;
    static final int DS_MATCH_TIME = 6;
    static final int DS_EVENT = 7;
    static final int DS_MATCH_TYPE = 8;
    static final int DS_MATCH_NUMBER = 9;
    static final int DS_GAME_MESSAGE = 10;

    static final ReplayEncoder.Field[] JOYSTICK_FIELDS = joystickFields();
    static final int J_CONNECTED = 0;
    static final int J_NAME = 1;
    static final int J_TYPE = 2;
    static final int J_BUTTONS = 3;
    static final int J_AXIS_0 = 4;
    static final int J_POV_0 = J_AXIS_0 + RuntimeInputSnapshot.AXES;

    static final ReplayEncoder.Field[] LOCALIZATION_FIELDS = {
            new ReplayEncoder.Field("type", Wire.TYPE_STRING, ""),
            new ReplayEncoder.Field("count", Wire.TYPE_I64, ""),
            new ReplayEncoder.Field("kind", Wire.TYPE_STRING, ""),
            new ReplayEncoder.Field("timestampSeconds", Wire.TYPE_F64, "s"),
            new ReplayEncoder.Field("latencySeconds", Wire.TYPE_F64, "s"),
            new ReplayEncoder.Field("xMeters", Wire.TYPE_F64, "m"),
            new ReplayEncoder.Field("yMeters", Wire.TYPE_F64, "m"),
            new ReplayEncoder.Field("headingRadians", Wire.TYPE_F64, "rad"),
    };

    private ReplayInputFields() {
    }

    private static ReplayEncoder.Field[] joystickFields() {
        List<ReplayEncoder.Field> fields = new ArrayList<>();
        fields.add(new ReplayEncoder.Field("connected", Wire.TYPE_BOOL, ""));
        fields.add(new ReplayEncoder.Field("name", Wire.TYPE_STRING, ""));
        fields.add(new ReplayEncoder.Field("type", Wire.TYPE_I64, ""));
        fields.add(new ReplayEncoder.Field("buttons", Wire.TYPE_I64, ""));
        for (int axis = 0; axis < RuntimeInputSnapshot.AXES; axis++) {
            fields.add(new ReplayEncoder.Field("axis" + axis, Wire.TYPE_F64, ""));
        }
        for (int pov = 0; pov < RuntimeInputSnapshot.POVS; pov++) {
            fields.add(new ReplayEncoder.Field("pov" + pov, Wire.TYPE_I64, "deg"));
        }
        return fields.toArray(ReplayEncoder.Field[]::new);
    }

    static String joystickPrefix(int port) {
        return JOYSTICK + port + "/";
    }

    /** Reads the input of the current frame. Returns empty input when the file has no input signals. */
    static RuntimeInputSnapshot read(ReplayReader.Cursor cursor) {
        if (!cursor.has(DRIVER_STATION + "enabled")) {
            return RuntimeInputSnapshot.empty();
        }
        List<RuntimeInputSnapshot.Joystick> joysticks = new ArrayList<>(RuntimeInputSnapshot.JOYSTICK_PORTS);
        for (int port = 0; port < RuntimeInputSnapshot.JOYSTICK_PORTS; port++) {
            String base = joystickPrefix(port);
            double[] axes = new double[RuntimeInputSnapshot.AXES];
            for (int axis = 0; axis < axes.length; axis++) {
                axes[axis] = cursor.f64(base + "axis" + axis);
            }
            int[] povs = new int[RuntimeInputSnapshot.POVS];
            for (int pov = 0; pov < povs.length; pov++) {
                povs[pov] = (int) cursor.i64(base + "pov" + pov);
            }
            joysticks.add(new RuntimeInputSnapshot.Joystick(
                    port,
                    cursor.bool(base + "connected"),
                    cursor.string(base + "name"),
                    (int) cursor.i64(base + "type"),
                    axes,
                    cursor.i64(base + "buttons"),
                    povs));
        }
        String ds = DRIVER_STATION;
        RuntimeInputSnapshot.DriverStationState state = new RuntimeInputSnapshot.DriverStationState(
                cursor.bool(ds + "enabled"),
                cursor.bool(ds + "autonomous"),
                cursor.bool(ds + "test"),
                cursor.bool(ds + "fmsAttached"),
                cursor.bool(ds + "dsAttached"),
                cursor.string(ds + "alliance"),
                cursor.f64(ds + "matchTime"),
                cursor.string(ds + "eventName"),
                cursor.string(ds + "matchType"),
                (int) cursor.i64(ds + "matchNumber"),
                cursor.string(ds + "gameMessage"));
        return new RuntimeInputSnapshot(joysticks, state);
    }
}
