package ca.frc6390.athena.wpilib.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.arcp.ArcpCycle;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class ArcpSignalsTest {

    private static final class Probe implements Mechanism {
        double reading = 1.0;
        final AtomicInteger reads = new AtomicInteger();

        @Telemetry
        private final TelemetryValue label = TelemetryValue.constant("probe");

        @Telemetry
        private final TelemetryValue value = TelemetryValue.number(() -> {
            reads.incrementAndGet();
            return reading;
        });
    }

    /** A system source with fixed values, so only the telemetry values under test change. */
    private static final class QuietSource implements ArcpSystemSignals.Source {
        @Override public double batteryVolts() { return 12.0; }
        @Override public String mode() { return "Disabled"; }
        @Override public double matchTimeSeconds() { return -1.0; }
        @Override public String alliance() { return ""; }
        @Override public double busDemandAmps() { return 0.0; }
        @Override public boolean busShedding() { return false; }
    }

    private static Map<Long, Object> sample(ArcpSignals signals) {
        ArcpCycle cycle = new ArcpCycle();
        signals.sample(cycle);
        Map<Long, Object> values = new HashMap<>();
        for (int i = 0; i < cycle.size(); i++) {
            values.put(cycle.signalId(i), cycle.value(i));
        }
        return values;
    }

    private static long id(ArcpSignals signals, String suffix) {
        return signals.dictionary().stream()
                .filter(entry -> entry.path().endsWith(suffix))
                .mapToLong(ArcpDictEntry::signalId)
                .findFirst()
                .orElseThrow();
    }

    @Test
    void forgetSentResendsCurrentValuesAfterAnIdleStretch() {
        Probe probe = new Probe();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(probe);
        ArcpSignals signals = new ArcpSignals(runtime.mechanismTelemetrySchema(), group -> false, new QuietSource());
        long value = id(signals, "/Values/value");

        sample(signals);
        assertFalse(sample(signals).containsKey(value), "an unchanged value is not resent");

        signals.forgetSent();
        assertEquals(1.0, (Double) sample(signals).get(value), 1e-12, "after forgetSent the value is sent again");
        assertFalse(sample(signals).containsKey(value));
    }

    @Test
    void constantsAreSentOnceAndChangedValuesInTheCycleTheyChange() {
        Probe probe = new Probe();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(probe);
        ArcpSignals signals = new ArcpSignals(runtime.mechanismTelemetrySchema(), group -> false, new QuietSource());
        long label = id(signals, "/Values/label");
        long value = id(signals, "/Values/value");

        Map<Long, Object> first = sample(signals);
        assertEquals("probe", first.get(label));
        assertEquals(1.0, (Double) first.get(value), 1e-12);

        int reads = probe.reads.get();
        Map<Long, Object> idle = sample(signals);
        assertFalse(idle.containsKey(label), "a constant is not resent");
        assertFalse(idle.containsKey(value), "an unchanged value is not resent");
        assertEquals(reads + 1, probe.reads.get(), "a non-constant value is still read every cycle");

        probe.reading = 2.5;
        Map<Long, Object> changed = sample(signals);
        assertEquals(2.5, (Double) changed.get(value), 1e-12);
        assertFalse(changed.containsKey(label));
        assertTrue(signals.valueCount() >= 2);
    }
}
