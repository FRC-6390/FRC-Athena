package ca.frc6390.athena.wpilib.sysid;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.DataLogManager;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class WpilibSysIdLogFormatTest {
    private static final String ROUTINE = "athena-format";
    private static final String MOTOR = "motor_1";

    @Test
    void emittedWpilogContainsCompleteFiniteAnalyzerInputs() throws Exception {
        Path directory = Path.of("build", "tmp", "sysid-format").toAbsolutePath();
        Files.createDirectories(directory);
        Path file = directory.resolve("sysid-test-" + System.nanoTime() + ".wpilog");
        DataLogManager.start(directory.toString(), file.getFileName().toString(), 0.01);
        try {
            SysIdLog log = new WpilibSysIdLogProvider().open(ROUTINE, MOTOR);
            for (SysIdState state : List.of(
                    SysIdState.QUASISTATIC_FORWARD,
                    SysIdState.QUASISTATIC_REVERSE,
                    SysIdState.DYNAMIC_FORWARD,
                    SysIdState.DYNAMIC_REVERSE)) {
                for (int sample = 0; sample < 3; sample++) {
                    double direction = state == SysIdState.QUASISTATIC_REVERSE
                                    || state == SysIdState.DYNAMIC_REVERSE
                            ? -1.0
                            : 1.0;
                    log.record(new SysIdSample(
                            state,
                            true,
                            direction * (sample + 1.0),
                            direction * sample * 0.1,
                            direction * sample * 0.2,
                            4.0 + sample));
                    Thread.sleep(2);
                }
                log.end();
            }
            DataLogManager.getLog().flush();
            Thread.sleep(50);
        } finally {
            DataLogManager.stop();
        }

        DataLogReader reader = new DataLogReader(file.toString());
        assertTrue(reader.isValid());

        Map<Integer, String> entries = new HashMap<>();
        Map<String, List<Double>> numeric = new HashMap<>();
        List<String> states = new ArrayList<>();
        Map<String, List<Long>> stateTimestamps = new HashMap<>();
        for (var record : reader) {
            if (record.isStart()) {
                var start = record.getStartData();
                entries.put(start.entry, start.name);
                continue;
            }
            String name = entries.get(record.getEntry());
            if (name == null) continue;
            if (name.equals("sysid-test-state-" + ROUTINE)) {
                String state = record.getString();
                states.add(state);
                stateTimestamps.computeIfAbsent(state, ignored -> new ArrayList<>())
                        .add(record.getTimestamp());
            } else if (name.endsWith("-" + MOTOR + "-" + ROUTINE)) {
                double value = record.getDouble();
                assertTrue(Double.isFinite(value), () -> name + " contains a non-finite sample");
                numeric.computeIfAbsent(name, ignored -> new ArrayList<>()).add(value);
            }
        }

        assertEquals(12, numeric.get("voltage-" + MOTOR + "-" + ROUTINE).size());
        assertEquals(12, numeric.get("position-" + MOTOR + "-" + ROUTINE).size());
        assertEquals(12, numeric.get("velocity-" + MOTOR + "-" + ROUTINE).size());
        for (SysIdState state : List.of(
                SysIdState.QUASISTATIC_FORWARD,
                SysIdState.QUASISTATIC_REVERSE,
                SysIdState.DYNAMIC_FORWARD,
                SysIdState.DYNAMIC_REVERSE)) {
            String name = wpilibState(state);
            assertEquals(3, states.stream().filter(value -> value.equals(name)).count());
            List<Long> timestamps = stateTimestamps.get(name);
            assertTrue(timestamps.get(timestamps.size() - 1) > timestamps.get(0),
                    () -> name + " has a zero-duration sample range");
        }
        assertTrue(states.stream().filter("none"::equals).count() >= 3);
    }

    private static String wpilibState(SysIdState state) {
        return switch (state) {
            case QUASISTATIC_FORWARD -> "quasistatic-forward";
            case QUASISTATIC_REVERSE -> "quasistatic-reverse";
            case DYNAMIC_FORWARD -> "dynamic-forward";
            case DYNAMIC_REVERSE -> "dynamic-reverse";
            case NONE -> "none";
        };
    }
}
