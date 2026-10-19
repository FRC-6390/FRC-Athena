package ca.frc6390.athena.wpilib.arcp;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.localization.pipeline.VisionSourceStatus;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class ArcpSystemSignalsTest {

    /** A source whose values the test sets directly. */
    private static final class FakeSource implements ArcpSystemSignals.Source {
        double volts = 12.5;
        String mode = "Disabled";
        double matchTime = -1;
        String alliance = "";
        double demand = 3.0;
        boolean shedding;
        List<PowerSnapshot.Motor> motors = List.of();
        List<VisionSourceStatus> vision = List.of();

        @Override public List<PowerSnapshot.Motor> motors() { return motors; }
        @Override public List<VisionSourceStatus> visionSources() { return vision; }
        @Override public double batteryVolts() { return volts; }
        @Override public String mode() { return mode; }
        @Override public double matchTimeSeconds() { return matchTime; }
        @Override public String alliance() { return alliance; }
        @Override public double busDemandAmps() { return demand; }
        @Override public boolean busShedding() { return shedding; }
    }

    private static Map<String, ArcpDictEntry> byPath(ArcpSystemSignals signals) {
        return signals.dictionary().stream().collect(Collectors.toMap(ArcpDictEntry::path, e -> e));
    }

    private static Map<Long, Object> sampled(ArcpSystemSignals signals) {
        ArcpCycle cycle = new ArcpCycle();
        signals.sample(cycle);
        Map<Long, Object> values = new java.util.HashMap<>();
        for (int i = 0; i < cycle.size(); i++) {
            values.put(cycle.signalId(i), cycle.value(i));
        }
        return values;
    }

    @Test
    void dictionaryUsesTheDashboardPathsWithSystemRoleAndUnits() {
        ArcpSystemSignals signals = new ArcpSystemSignals(40, new FakeSource());
        Map<String, ArcpDictEntry> entries = byPath(signals);

        assertEquals(
                List.of(
                        "athena/robot/mode",
                        "athena/robot/matchTime",
                        "athena/robot/alliance",
                        "athena/power/battery/voltage",
                        "athena/power/bus/demandAmps",
                        "athena/power/bus/shedding"),
                signals.dictionary().stream().map(ArcpDictEntry::path).toList());
        for (ArcpDictEntry entry : signals.dictionary()) {
            assertEquals(ArcpDictEntry.ROLE_SYSTEM, entry.role(), entry.path());
            assertFalse(entry.writable(), entry.path());
        }
        assertEquals("V", entries.get("athena/power/battery/voltage").unit());
        assertEquals("A", entries.get("athena/power/bus/demandAmps").unit());
        assertEquals("s", entries.get("athena/robot/matchTime").unit());
        assertEquals(ArcpDictEntry.ROLE_SYSTEM, entries.get("athena/robot/mode").role());
    }

    @Test
    void idsStartAtTheFirstIdAndTheNextIdFollowsThem() {
        ArcpSystemSignals signals = new ArcpSystemSignals(40, new FakeSource());
        assertEquals(40L, signals.dictionary().get(0).signalId());
        assertEquals(45L, signals.dictionary().get(5).signalId());
        assertEquals(46L, signals.nextId());
    }

    @Test
    void samplesPublishOnlyChangedValues() {
        FakeSource source = new FakeSource();
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        Map<String, ArcpDictEntry> entries = byPath(signals);

        Map<Long, Object> first = sampled(signals);
        assertEquals(6, first.size());
        assertEquals("Disabled", first.get(entries.get("athena/robot/mode").signalId()));
        assertEquals(12.5, (Double) first.get(entries.get("athena/power/battery/voltage").signalId()), 1e-12);
        assertEquals(Boolean.FALSE, first.get(entries.get("athena/power/bus/shedding").signalId()));

        assertTrue(sampled(signals).isEmpty(), "nothing changed, nothing sent");

        source.mode = "Teleop";
        source.matchTime = 120.5;
        Map<Long, Object> changed = sampled(signals);
        assertEquals(Map.of(
                entries.get("athena/robot/mode").signalId(), "Teleop",
                entries.get("athena/robot/matchTime").signalId(), 120.5), changed);
    }

    @Test
    void nonFiniteNumbersAreSkippedAndTheLastValueStands() {
        FakeSource source = new FakeSource();
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        ArcpDictEntry battery = byPath(signals).get("athena/power/battery/voltage");
        sampled(signals);

        source.volts = Double.NaN;
        assertFalse(sampled(signals).containsKey(battery.signalId()));
        source.volts = 11.0;
        assertEquals(11.0, (Double) sampled(signals).get(battery.signalId()), 1e-12);
    }

    @Test
    void missingAllianceAndModeArePublishedAsEmptyText() {
        FakeSource source = new FakeSource();
        source.alliance = null;
        source.mode = null;
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        Map<Long, Object> values = sampled(signals);
        ArcpDictEntry alliance = byPath(signals).get("athena/robot/alliance");
        assertEquals("", values.get(alliance.signalId()));
    }

    @Test
    void dictionaryGrowsWhenThePowerPassReportsNewMotorsAndExistingIdsStay() {
        FakeSource source = new FakeSource();
        ArcpSystemSignals signals = new ArcpSystemSignals(40, source);
        Map<String, ArcpDictEntry> before = byPath(signals);
        assertFalse(signals.discoverMotors(), "no motors before the first power cycle");
        assertEquals(6, signals.dictionary().size());

        source.motors = List.of(motor("drive/left", 30.0, Double.POSITIVE_INFINITY));
        assertTrue(signals.discoverMotors(), "the first reported motor grows the dictionary");
        assertFalse(signals.discoverMotors(), "the same motor again adds nothing");
        assertEquals(8, signals.dictionary().size());
        assertEquals(48L, signals.nextId());

        Map<String, ArcpDictEntry> after = byPath(signals);
        for (ArcpDictEntry entry : before.values()) {
            assertEquals(entry.signalId(), after.get(entry.path()).signalId(), entry.path());
        }
        ArcpDictEntry current = after.get("athena/power/drive/left/current");
        ArcpDictEntry cap = after.get("athena/power/drive/left/capAmps");
        assertEquals(46L, current.signalId());
        assertEquals(47L, cap.signalId());
        assertEquals("A", current.unit());
        assertEquals("A", cap.unit());
        assertEquals(ArcpDictEntry.ROLE_SYSTEM, current.role());
        assertFalse(current.writable());

        source.motors = List.of(motor("drive/left", 30.0, Double.POSITIVE_INFINITY),
                motor("drive/right", 12.0, 40.0));
        assertTrue(signals.discoverMotors());
        assertEquals(10, signals.dictionary().size());
        assertEquals(48L, byPath(signals).get("athena/power/drive/right/current").signalId());
    }

    @Test
    void motorChannelsSampleCurrentAndCapAndZeroMeansNoCap() {
        FakeSource source = new FakeSource();
        source.motors = List.of(motor("arm", 22.5, Double.POSITIVE_INFINITY));
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        signals.discoverMotors();
        Map<String, ArcpDictEntry> entries = byPath(signals);

        Map<Long, Object> first = sampled(signals);
        assertEquals(22.5, (Double) first.get(entries.get("athena/power/arm/current").signalId()), 1e-12);
        assertEquals(0.0, (Double) first.get(entries.get("athena/power/arm/capAmps").signalId()), 1e-12,
                "an uncapped motor reports 0, never infinity");

        source.motors = List.of(motor("arm", 22.5, 35.0));
        assertEquals(Map.of(entries.get("athena/power/arm/capAmps").signalId(), 35.0), sampled(signals));
    }

    private static PowerSnapshot.Motor motor(String name, double demandAmps, double capAmps) {
        return new PowerSnapshot.Motor(name, demandAmps, capAmps, 1.0, 25.0, 1.0, 0.0, "Declared");
    }

    private static VisionSourceStatus camera(String name, VisionSourceStatus.State state, String reason,
            double accepted, double rejected, double frameAgeSeconds, int tags, double secondsSinceAccepted) {
        return new VisionSourceStatus(name, state, reason, Hertz.of(accepted), Hertz.of(rejected),
                Seconds.of(frameAgeSeconds), tags, Seconds.of(secondsSinceAccepted));
    }

    @Test
    void visionChannelsAppearPerCameraWithTheDashboardPathsTypesAndUnits() {
        FakeSource source = new FakeSource();
        ArcpSystemSignals signals = new ArcpSystemSignals(40, source);
        assertFalse(signals.discoverVision(), "no cameras before a Kalman localization has judged a frame");

        source.vision = List.of(camera("Intake", VisionSourceStatus.State.REJECTED, "ambiguity 0.31 > 0.20",
                2.0, 3.0, 0.02, 1, 0.4));
        assertTrue(signals.discoverVision());
        assertFalse(signals.discoverVision(), "the same camera again adds nothing");

        Map<String, ArcpDictEntry> entries = byPath(signals);
        assertEquals(List.of(
                "athena/vision/Intake/State",
                "athena/vision/Intake/Reason",
                "athena/vision/Intake/AcceptedPerSecond",
                "athena/vision/Intake/RejectedPerSecond",
                "athena/vision/Intake/FrameAgeMs",
                "athena/vision/Intake/TagCount",
                "athena/vision/Intake/SecondsSinceAccepted",
                "athena/vision/Intake/TimeSync"),
                signals.dictionary().stream().map(ArcpDictEntry::path)
                        .filter(path -> path.startsWith("athena/vision/")).toList());
        assertEquals(4, entries.get("athena/vision/Intake/State").typeTag());
        assertEquals(4, entries.get("athena/vision/Intake/Reason").typeTag());
        assertEquals(3, entries.get("athena/vision/Intake/AcceptedPerSecond").typeTag());
        assertEquals(2, entries.get("athena/vision/Intake/TagCount").typeTag(), "TagCount is i64");
        assertEquals(1, entries.get("athena/vision/Intake/TimeSync").typeTag(), "TimeSync is bool");
        assertEquals("ms", entries.get("athena/vision/Intake/FrameAgeMs").unit());
        assertEquals("Hz", entries.get("athena/vision/Intake/AcceptedPerSecond").unit());
        for (String path : entries.keySet()) {
            if (path.startsWith("athena/vision/")) {
                assertEquals(ArcpDictEntry.ROLE_SYSTEM, entries.get(path).role(), path);
                assertFalse(entries.get(path).writable(), path);
            }
        }
    }

    @Test
    void timeSyncIsSentOnlyForCamerasThatKnowTheirClockAndOnlyOnChange() {
        FakeSource source = new FakeSource();
        source.vision = List.of(new VisionSourceStatus("Intake", VisionSourceStatus.State.UNSYNCED, "frame 3.0 s in the future",
                Hertz.of(0.0), Hertz.of(5.0), Seconds.of(-3.0), 1, Seconds.of(Double.NaN), false));
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        signals.discoverVision();
        long timeSync = byPath(signals).get("athena/vision/Intake/TimeSync").signalId();

        assertEquals(false, sampled(signals).get(timeSync));
        assertFalse(sampled(signals).containsKey(timeSync), "unchanged");
        source.vision = List.of(new VisionSourceStatus("Intake", VisionSourceStatus.State.OK, "",
                Hertz.of(5.0), Hertz.of(0.0), Seconds.of(0.02), 2, Seconds.of(0.0), true));
        assertEquals(true, sampled(signals).get(timeSync));
        source.vision = List.of(new VisionSourceStatus("Intake", VisionSourceStatus.State.OK, "",
                Hertz.of(5.0), Hertz.of(0.0), Seconds.of(0.02), 2, Seconds.of(0.0)));
        assertFalse(sampled(signals).containsKey(timeSync), "unknown clock sends nothing");
    }

    @Test
    void visionSamplesPublishStateReasonRatesAgeAndTagCountOnChange() {
        FakeSource source = new FakeSource();
        source.vision = List.of(camera("Intake", VisionSourceStatus.State.REJECTED, "ambiguity 0.31 > 0.20",
                2.0, 3.0, 0.02, 1, 0.4));
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        signals.discoverVision();
        Map<String, ArcpDictEntry> entries = byPath(signals);

        Map<Long, Object> first = sampled(signals);
        assertEquals("REJECTED", first.get(entries.get("athena/vision/Intake/State").signalId()));
        assertEquals("ambiguity 0.31 > 0.20", first.get(entries.get("athena/vision/Intake/Reason").signalId()));
        assertEquals(2.0, (Double) first.get(entries.get("athena/vision/Intake/AcceptedPerSecond").signalId()), 1e-12);
        assertEquals(20.0, (Double) first.get(entries.get("athena/vision/Intake/FrameAgeMs").signalId()), 1e-9);
        assertEquals(1L, first.get(entries.get("athena/vision/Intake/TagCount").signalId()));
        assertEquals(0.4, (Double) first.get(entries.get("athena/vision/Intake/SecondsSinceAccepted").signalId()), 1e-12);

        assertTrue(sampled(signals).isEmpty(), "unchanged values are not sent again");

        source.vision = List.of(camera("Intake", VisionSourceStatus.State.STALE, "no frame for 0.8 s",
                2.0, 3.0, Double.NaN, 1, Double.NaN));
        Map<Long, Object> changed = sampled(signals);
        assertEquals("STALE", changed.get(entries.get("athena/vision/Intake/State").signalId()));
        assertFalse(changed.containsKey(entries.get("athena/vision/Intake/FrameAgeMs").signalId()),
                "a NaN frame age is not sent; the last value stands");
    }

    @Test
    void aSecondCameraAppendsItsOwnIdsAndTheFirstCameraKeepsItsIds() {
        FakeSource source = new FakeSource();
        source.vision = List.of(camera("Intake", VisionSourceStatus.State.OK, "", 2.0, 0.0, 0.01, 2, 0.0));
        ArcpSystemSignals signals = new ArcpSystemSignals(1, source);
        signals.discoverVision();
        long intakeState = byPath(signals).get("athena/vision/Intake/State").signalId();

        source.vision = List.of(
                camera("Intake", VisionSourceStatus.State.OK, "", 2.0, 0.0, 0.01, 2, 0.0),
                camera("Turret", VisionSourceStatus.State.UNSYNCED, "frame 3.2 s in the future", 0.0, 4.0, -3.2, 1,
                        Double.NaN));
        assertTrue(signals.discoverVision());

        Map<String, ArcpDictEntry> entries = byPath(signals);
        assertEquals(intakeState, entries.get("athena/vision/Intake/State").signalId());
        assertEquals(intakeState + 8, entries.get("athena/vision/Turret/State").signalId());
        assertEquals(intakeState + 16, signals.nextId(), "the second camera takes the next eight ids");
    }
}
