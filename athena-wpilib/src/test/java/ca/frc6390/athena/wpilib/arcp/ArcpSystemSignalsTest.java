package ca.frc6390.athena.wpilib.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
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

        @Override public List<PowerSnapshot.Motor> motors() { return motors; }
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
}
