package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.List;
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
 * </ul>
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
            };
        }
    }

    private final Source source;
    private final List<ArcpDictEntry> dictionary;
    private final long modeId;
    private final long matchTimeId;
    private final long allianceId;
    private final long batteryId;
    private final long demandId;
    private final long sheddingId;
    private final long nextId;

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
        dictionary = List.of(
                entry(modeId, MODE, TYPE_STRING, ""),
                entry(matchTimeId, MATCH_TIME, TYPE_F64, "s"),
                entry(allianceId, ALLIANCE, TYPE_STRING, ""),
                entry(batteryId, BATTERY_VOLTS, TYPE_F64, "V"),
                entry(demandId, BUS_DEMAND_AMPS, TYPE_F64, "A"),
                entry(sheddingId, BUS_SHEDDING, TYPE_BOOL, ""));
    }

    private static ArcpDictEntry entry(long id, String path, int typeTag, String unit) {
        return new ArcpDictEntry(id, path, typeTag, unit, ArcpDictEntry.ROLE_SYSTEM, 0, "", 0);
    }

    /** The dictionary entries, in signal id order. */
    List<ArcpDictEntry> dictionary() {
        return dictionary;
    }

    /** The first id not used by these entries. */
    long nextId() {
        return nextId;
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
    }

    private static String text(String value) {
        return value == null ? "" : value;
    }
}
