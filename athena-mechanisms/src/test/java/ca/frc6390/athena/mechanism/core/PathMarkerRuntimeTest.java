package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

/** Path marker lifecycle: completion, shared-resource precedence, and evaluation order. */
class PathMarkerRuntimeTest {
    private static final MotorDevice DRIVE = MotorDevice.of(MotorKinds.KRAKEN_X60, 51);
    private static final MotorDevice ROLLERS = MotorDevice.of(MotorKinds.KRAKEN_X44, 52);

    @Test
    void finiteMarkerActionStopsOnceCompleteWhileContinuousMarkerKeepsRunning() {
        AtomicInteger runs = new AtomicInteger();
        RecordingContext hardware = new RecordingContext();
        Robot robot = new Robot();
        Action once = Actions.doOnce(runs::incrementAndGet);
        PathAction path = new PathAction("test", "Markers").marker("count", once).marker("spin", ROLLERS.percent(0.4));
        MarkerPath runtime = new MarkerPath(1.0);
        runtime.schedule(0.0, "count", once);
        runtime.schedule(0.0, "spin", path.markers().get("spin"));
        MechanismScheduler scheduler = scheduler(hardware, robot, path, runtime);

        scheduler.request(path);
        for (int cycle = 0; cycle < 10; cycle++) scheduler.autoPeriodic(cycle * 0.02, 0.02);

        assertEquals(1, runs.get());
        assertEquals(List.of("count"), runtime.completed);
        assertEquals(0.4, hardware.motor(ROLLERS).percent, 1.0e-9);
    }

    @Test
    void laterTriggeredMarkerWinsASharedMotorWithoutAnOutputConflict() {
        RecordingContext hardware = new RecordingContext();
        Robot robot = new Robot();
        PathAction path = new PathAction("test", "Overlap")
                .marker("intake", ROLLERS.percent(0.6))
                .marker("eject", ROLLERS.percent(-0.3));
        MarkerPath runtime = new MarkerPath(1.0);
        runtime.schedule(0.0, "intake", path.markers().get("intake"));
        runtime.schedule(0.1, "eject", path.markers().get("eject"));
        MechanismScheduler scheduler = scheduler(hardware, robot, path, runtime);

        scheduler.request(path);
        scheduler.autoPeriodic(0.0, 0.02);
        assertEquals(0.6, hardware.motor(ROLLERS).percent, 1.0e-9);
        assertDoesNotThrow(() -> {
            for (int cycle = 1; cycle < 10; cycle++) scheduler.autoPeriodic(cycle * 0.02, 0.02);
        });
        assertEquals(-0.3, hardware.motor(ROLLERS).percent, 1.0e-9);
        assertEquals(0.5, hardware.motor(DRIVE).percent, 1.0e-9, "path drive output is unaffected");
    }

    @Test
    void markersAreEvaluatedBeforeThePathOutputIsRequested() {
        double[] pose = {7.0};
        RecordingContext hardware = new RecordingContext();
        Robot robot = new Robot();
        PathAction path = new PathAction("test", "Reset");
        MarkerPath runtime = new MarkerPath(1.0) {
            @Override
            public Action output(PathAction ignored, MechanismContext context) {
                outputPoses.add(pose[0]);
                return super.output(ignored, context);
            }
        };
        runtime.schedule(0.0, "@reset", Actions.doOnce(() -> pose[0] = 0.0));
        MechanismScheduler scheduler = scheduler(hardware, robot, path, runtime);

        scheduler.request(path);
        scheduler.autoPeriodic(0.0, 0.02);

        assertEquals(List.of(0.0), runtime.outputPoses);
    }

    private static MechanismScheduler scheduler(
            RecordingContext hardware, Robot robot, PathAction path, PathRuntime runtime) {
        return MechanismScheduler.create(hardware)
                .register(robot)
                .path(path, runtime)
                .declarationOwner(path, robot);
    }

    private static final class Robot implements Mechanism {
        @SuppressWarnings("unused")
        private final MotorDevice drive = DRIVE;
        @SuppressWarnings("unused")
        private final MotorDevice rollers = ROLLERS;
    }

    /** Test provider that triggers named markers at fixed times and drives DRIVE at 0.5. */
    private static class MarkerPath implements PathRuntime {
        private final double seconds;
        private final List<Object[]> scheduled = new ArrayList<>();
        private final Map<String, Action> active = new LinkedHashMap<>();
        private final List<String> completed = new ArrayList<>();
        protected final List<Double> outputPoses = new ArrayList<>();
        private final Action drive = DRIVE.percent(0.5);

        private MarkerPath(double seconds) {
            this.seconds = seconds;
        }

        private void schedule(double time, String name, Action action) {
            scheduled.add(new Object[] {time, name, action});
        }

        @Override
        public void initialize(PathAction path, MechanismContext context) {
            active.clear();
        }

        @Override
        public void execute(PathAction path, MechanismContext context) {
            for (Object[] marker : scheduled) {
                if ((double) marker[0] <= context.timeInStateSeconds() + 1.0e-9 && !completed.contains(marker[1])) {
                    active.putIfAbsent((String) marker[1], (Action) marker[2]);
                }
            }
        }

        @Override
        public Action output(PathAction path, MechanismContext context) {
            return drive;
        }

        @Override
        public List<?> ownership(PathAction path) {
            return List.of(DRIVE);
        }

        @Override
        public Map<String, Action> activeMarkers(PathAction path, MechanismContext context) {
            return new LinkedHashMap<>(active);
        }

        @Override
        public void markerCompleted(PathAction path, String marker) {
            completed.add(marker);
            active.remove(marker);
        }

        @Override
        public boolean isFinished(PathAction path, MechanismContext context) {
            return context.timeInStateSeconds() >= seconds;
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final Map<MotorDevice, RecordingMotor> motors = new HashMap<>();

        @Override
        public RecordingMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, RecordingMotor::new);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double percent = Double.NaN;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public void setPercentOutput(double percent) { this.percent = percent; }
        @Override public void stop() { percent = 0.0; }
    }
}
