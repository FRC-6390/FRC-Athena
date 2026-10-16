package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.localization.pipeline.VisionSourceStatus;
import ca.frc6390.athena.localization.pipeline.VisionStatusRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Robot-level system signals for the dashboard: the DriverStation mode, match clock and alliance, the battery
 * voltage, and the power pass's bus demand. They carry role {@link ArcpDictEntry#ROLE_SYSTEM}, sit outside the
 * mechanism tree, and are read-only.
 *
 * <p>Paths (the host dashboard reads these as {@code ROBOT_PATHS} in {@code src/lib/data/types.ts}):</p>
 * <ul>
 *   <li>{@code athena/robot/mode}: string, {@code Disabled}, {@code Autonomous}, {@code Teleop} or {@code Test};</li>
 *   <li>{@code athena/robot/matchTime}: f64, seconds left in the current period, -1 when none is running;</li>
 *   <li>{@code athena/robot/alliance}: string, {@code Red}, {@code Blue}, or empty when unknown;</li>
 *   <li>{@code athena/power/battery/voltage}: f64, volts;</li>
 *   <li>{@code athena/power/bus/demandAmps}: f64, total requested current from the power pass;</li>
 *   <li>{@code athena/power/bus/shedding}: bool, true while the power budget sheds current.</li>
 *   <li>{@code athena/power/<motor>/current}: f64, amps: the motor's estimated current draw;</li>
 *   <li>{@code athena/power/<motor>/capAmps}: f64, amps: the planner's cap on the motor, 0 when it is not capped.</li>
 *   <li>{@code athena/vision/<camera>/State}: string, OK, REJECTED, RECOVERING, UNSYNCED, STALE or NO_TARGETS;</li>
 *   <li>{@code athena/vision/<camera>/Reason}: string, short text naming the rejection, empty when OK;</li>
 *   <li>{@code athena/vision/<camera>/AcceptedPerSecond}, {@code RejectedPerSecond}: f64, frames per second;</li>
 *   <li>{@code athena/vision/<camera>/FrameAgeMs}: f64, milliseconds since the newest frame was captured;</li>
 *   <li>{@code athena/vision/<camera>/TagCount}: i64, tags in the newest frame;</li>
 *   <li>{@code athena/vision/<camera>/SecondsSinceAccepted}: f64, seconds since the last accepted frame.</li>
 * </ul>
 *
 * <p>The per-motor paths exist only once the power pass has reported the motor, which happens after its first cycle.
 * The per-camera vision paths exist once a Kalman localization has judged a frame from that camera. Both are added by
 * {@link #discoverMotors()} and {@link #discoverVision()}; existing ids never change. Entries are only ever added.</p>
 *
 * <p>Numbers that are not finite are not published; clients keep the last value they received. Sampling runs on
 * the robot thread and publishes only the values that changed.</p>
 */
final class ArcpSystemSignals {

    static final String MODE = "athena/robot/mode";
    static final String MATCH_TIME = "athena/robot/matchTime";
    static final String ALLIANCE = "athena/robot/alliance";
    static final String BATTERY_VOLTS = "athena/power/battery/voltage";
    static final String BUS_DEMAND_AMPS = "athena/power/bus/demandAmps";
    static final String BUS_SHEDDING = "athena/power/bus/shedding";

    private static final int TYPE_BOOL = 1;
    private static final int TYPE_I64 = 2;
    private static final int TYPE_F64 = 3;
    private static final int TYPE_STRING = 4;

    /** Where the values come from. The WPILib implementation is {@link #wpilib}; tests pass their own. */
    interface Source {
        double batteryVolts();

        String mode();

        double matchTimeSeconds();

        String alliance();

        double busDemandAmps();

        boolean busShedding();

        /** The power pass's per-motor state. Empty until the power pass has cycled. */
        default List<PowerSnapshot.Motor> motors() {
            return List.of();
        }

        /** The vision sources' statuses from the Kalman localizations. Empty until one has refreshed. */
        default List<VisionSourceStatus> visionSources() {
            return List.of();
        }

        /**
         * The live robot: RobotController for the battery, DriverStation for mode, match clock and alliance, and the
         * power pass for the bus.
         *
         * @param power latest electrical state from the robot runtime
         * @return source
         */
        static Source wpilib(Supplier<PowerSnapshot> power) {
            Objects.requireNonNull(power, "power");
            return new Source() {
                @Override
                public double batteryVolts() {
                    return RobotController.getBatteryVoltage();
                }

                @Override
                public String mode() {
                    if (DriverStation.isAutonomousEnabled()) return "Autonomous";
                    if (DriverStation.isTeleopEnabled()) return "Teleop";
                    if (DriverStation.isTestEnabled()) return "Test";
                    return "Disabled";
                }

                @Override
                public double matchTimeSeconds() {
                    return DriverStation.getMatchTime();
                }

                @Override
                public String alliance() {
                    return DriverStation.getAlliance()
                            .map(alliance -> alliance == DriverStation.Alliance.Red ? "Red" : "Blue")
                            .orElse("");
                }

                @Override
                public double busDemandAmps() {
                    return power.get().demandAmps();
                }

                @Override
                public boolean busShedding() {
                    return power.get().shedding();
                }

                @Override
                public List<PowerSnapshot.Motor> motors() {
                    return power.get().motors();
                }

                @Override
                public List<VisionSourceStatus> visionSources() {
                    return VisionStatusRegistry.latest();
                }
            };
        }
    }

    /** One motor's channels, keyed by the motor name the power pass reports. */
    private static final class MotorChannel {
        final long currentId;
        final long capId;
        double lastCurrent = Double.NaN;
        double lastCap = Double.NaN;

        MotorChannel(long currentId, long capId) {
            this.currentId = currentId;
            this.capId = capId;
        }
    }

    /** One camera's published channels and the last values sent for them. */
    private static final class VisionChannel {
        final long stateId;
        final long reasonId;
        final long acceptedId;
        final long rejectedId;
        final long frameAgeId;
        final long tagCountId;
        final long secondsId;
        String lastState;
        String lastReason;
        double lastAccepted = Double.NaN;
        double lastRejected = Double.NaN;
        double lastFrameAgeMs = Double.NaN;
        Long lastTagCount;
        double lastSeconds = Double.NaN;

        VisionChannel(long firstId) {
            stateId = firstId;
            reasonId = firstId + 1;
            acceptedId = firstId + 2;
            rejectedId = firstId + 3;
            frameAgeId = firstId + 4;
            tagCountId = firstId + 5;
            secondsId = firstId + 6;
        }
    }

    private static final int VISION_SIGNALS_PER_CAMERA = 7;
    private static final String POWER_PREFIX = "athena/power/";
    private static final String VISION_PREFIX = "athena/vision/";
    private final Map<String, VisionChannel> vision = new HashMap<>();

    private final Source source;
    private final List<ArcpDictEntry> dictionary;
    private final Map<String, MotorChannel> motors = new HashMap<>();
    private final long modeId;
    private final long matchTimeId;
    private final long allianceId;
    private final long batteryId;
    private final long demandId;
    private final long sheddingId;
    private long nextId;

    private String lastMode;
    private String lastAlliance;
    private double lastMatchTime = Double.NaN;
    private double lastBatteryVolts = Double.NaN;
    private double lastDemandAmps = Double.NaN;
    private Boolean lastShedding;

    /**
     * Builds the entries, numbered from {@code firstId}.
     *
     * @param firstId first signal id to use
     * @param source value source
     */
    ArcpSystemSignals(long firstId, Source source) {
        this.source = Objects.requireNonNull(source, "source");
        long id = firstId;
        modeId = id++;
        matchTimeId = id++;
        allianceId = id++;
        batteryId = id++;
        demandId = id++;
        sheddingId = id++;
        nextId = id;
        dictionary = new ArrayList<>(List.of(
                entry(modeId, MODE, TYPE_STRING, ""),
                entry(matchTimeId, MATCH_TIME, TYPE_F64, "s"),
                entry(allianceId, ALLIANCE, TYPE_STRING, ""),
                entry(batteryId, BATTERY_VOLTS, TYPE_F64, "V"),
                entry(demandId, BUS_DEMAND_AMPS, TYPE_F64, "A"),
                entry(sheddingId, BUS_SHEDDING, TYPE_BOOL, "")));
    }

    private static ArcpDictEntry entry(long id, String path, int typeTag, String unit) {
        return new ArcpDictEntry(id, path, typeTag, unit, ArcpDictEntry.ROLE_SYSTEM, 0, "", 0);
    }

    /**
     * Adds the seven signals of every camera the Kalman localizations report that is not known yet. Returns true when
     * the dictionary grew. Reads the vision statuses, so call it at the export cadence, not every cycle.
     *
     * @return true when entries were added
     */
    boolean discoverVision() {
        boolean grew = false;
        for (VisionSourceStatus status : source.visionSources()) {
            if (vision.containsKey(status.name())) {
                continue;
            }
            VisionChannel channel = new VisionChannel(nextId);
            nextId += VISION_SIGNALS_PER_CAMERA;
            vision.put(status.name(), channel);
            String prefix = VISION_PREFIX + status.name() + "/";
            dictionary.add(entry(channel.stateId, prefix + "State", TYPE_STRING, ""));
            dictionary.add(entry(channel.reasonId, prefix + "Reason", TYPE_STRING, ""));
            dictionary.add(entry(channel.acceptedId, prefix + "AcceptedPerSecond", TYPE_F64, "Hz"));
            dictionary.add(entry(channel.rejectedId, prefix + "RejectedPerSecond", TYPE_F64, "Hz"));
            dictionary.add(entry(channel.frameAgeId, prefix + "FrameAgeMs", TYPE_F64, "ms"));
            dictionary.add(entry(channel.tagCountId, prefix + "TagCount", TYPE_I64, ""));
            dictionary.add(entry(channel.secondsId, prefix + "SecondsSinceAccepted", TYPE_F64, "s"));
            // TODO: TimeSync (bool, camera clock synced to robot time) goes here once the status exists.
            grew = true;
        }
        return grew;
    }

    /** The dictionary entries, in signal id order. */
    List<ArcpDictEntry> dictionary() {
        return List.copyOf(dictionary);
    }

    /** The first id not used by these entries. */
    long nextId() {
        return nextId;
    }

    /**
     * Adds the current and cap entries of every motor the power pass reports that is not known yet. Returns true when
     * the dictionary grew, so the caller republishes it. Reads the power snapshot, so call it at the export cadence,
     * not every cycle.
     *
     * @return true when entries were added
     */
    boolean discoverMotors() {
        boolean grew = false;
        for (PowerSnapshot.Motor motor : source.motors()) {
            if (motors.containsKey(motor.name())) {
                continue;
            }
            long currentId = nextId++;
            long capId = nextId++;
            motors.put(motor.name(), new MotorChannel(currentId, capId));
            dictionary.add(entry(currentId, POWER_PREFIX + motor.name() + "/current", TYPE_F64, "A"));
            dictionary.add(entry(capId, POWER_PREFIX + motor.name() + "/capAmps", TYPE_F64, "A"));
            grew = true;
        }
        return grew;
    }

    /**
     * Writes the values that changed since the last sample into {@code cycle}.
     *
     * @param cycle cycle being built
     */
    void sample(ArcpCycle cycle) {
        String mode = text(source.mode());
        if (!mode.equals(lastMode)) {
            cycle.set(modeId, mode);
            lastMode = mode;
        }
        String alliance = text(source.alliance());
        if (!alliance.equals(lastAlliance)) {
            cycle.set(allianceId, alliance);
            lastAlliance = alliance;
        }
        double matchTime = source.matchTimeSeconds();
        if (Double.isFinite(matchTime) && matchTime != lastMatchTime) {
            cycle.setNumber(matchTimeId, matchTime);
            lastMatchTime = matchTime;
        }
        double volts = source.batteryVolts();
        if (Double.isFinite(volts) && volts != lastBatteryVolts) {
            cycle.setNumber(batteryId, volts);
            lastBatteryVolts = volts;
        }
        double demand = source.busDemandAmps();
        if (Double.isFinite(demand) && demand != lastDemandAmps) {
            cycle.setNumber(demandId, demand);
            lastDemandAmps = demand;
        }
        boolean shedding = source.busShedding();
        if (lastShedding == null || shedding != lastShedding) {
            cycle.setBoolean(sheddingId, shedding);
            lastShedding = shedding;
        }
        if (!vision.isEmpty()) {
            sampleVision(cycle);
        }
        if (!motors.isEmpty()) {
            for (PowerSnapshot.Motor motor : source.motors()) {
                MotorChannel channel = motors.get(motor.name());
                if (channel == null) {
                    continue;
                }
                double current = motor.demandAmps();
                if (Double.isFinite(current) && current != channel.lastCurrent) {
                    cycle.setNumber(channel.currentId, current);
                    channel.lastCurrent = current;
                }
                // The planner cap is +infinity when the motor is not capped; 0 means "no cap" on the wire.
                double cap = Double.isFinite(motor.capAmps()) ? motor.capAmps() : 0.0;
                if (cap != channel.lastCap) {
                    cycle.setNumber(channel.capId, cap);
                    channel.lastCap = cap;
                }
            }
        }
    }

    private void sampleVision(ArcpCycle cycle) {
        for (VisionSourceStatus status : source.visionSources()) {
            VisionChannel channel = vision.get(status.name());
            if (channel == null) {
                continue;
            }
            String state = status.state().name();
            if (!state.equals(channel.lastState)) {
                cycle.set(channel.stateId, state);
                channel.lastState = state;
            }
            String reason = text(status.reason());
            if (!reason.equals(channel.lastReason)) {
                cycle.set(channel.reasonId, reason);
                channel.lastReason = reason;
            }
            channel.lastAccepted = publishIfChanged(cycle, channel.acceptedId, status.acceptedPerSecond(), channel.lastAccepted);
            channel.lastRejected = publishIfChanged(cycle, channel.rejectedId, status.rejectedPerSecond(), channel.lastRejected);
            channel.lastFrameAgeMs = publishIfChanged(cycle, channel.frameAgeId,
                    status.frameAgeSeconds() * 1000.0, channel.lastFrameAgeMs);
            if (channel.lastTagCount == null || channel.lastTagCount != status.tagCount()) {
                cycle.setLong(channel.tagCountId, status.tagCount());
                channel.lastTagCount = (long) status.tagCount();
            }
            channel.lastSeconds = publishIfChanged(cycle, channel.secondsId, status.secondsSinceAccepted(),
                    channel.lastSeconds);
        }
    }

    /** Writes a finite value that changed; returns the value to remember. Non-finite values are not sent. */
    private static double publishIfChanged(ArcpCycle cycle, long signalId, double value, double last) {
        if (!Double.isFinite(value) || value == last) {
            return last;
        }
        cycle.setNumber(signalId, value);
        return value;
    }

    private static String text(String value) {
        return value == null ? "" : value;
    }
}
