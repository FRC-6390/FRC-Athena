package ca.frc6390.athena.wpilib.lifecycle;

import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.replay.ReplayMarkers;
import ca.frc6390.athena.robot.replay.ReplayMatch;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.ReplayRecorder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.file.Path;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Connects {@link AthenaRobot} lifecycle callbacks and driver station state to the replay recorder.
 */
final class ReplayBridge {
    static final Path ROBOT_REPLAY_DIRECTORY = Path.of("/home/lvuser/athena/replays");

    private final ReplayRecorder recorder;
    private final Supplier<ReplayMatch> match;
    private ReplayMatch lastMatch = ReplayMatch.none();
    private boolean browned;

    ReplayBridge(RobotRuntime runtime, ReplayPolicy policy, Path directory, Supplier<ReplayMatch> match) {
        Objects.requireNonNull(runtime, "runtime");
        Objects.requireNonNull(policy, "policy");
        this.match = Objects.requireNonNull(match, "match");
        runtime.replay(policy, directory);
        this.recorder = runtime.replayRecorder().orElseThrow();
    }

    /**
     * Creates the bridge with driver station match information and brownout detection. The runtime also samples
     * joysticks and driver station state each cycle, so recordings can be replayed with the same input.
     */
    static ReplayBridge attach(RobotRuntime runtime, ReplayPolicy policy, boolean simulation) {
        Path directory = policy.directory().orElse(defaultDirectory(simulation));
        runtime.inputs(DriverStationInputs::sample);
        return new ReplayBridge(runtime, policy, directory, ReplayBridge::driverStationMatch);
    }

    /**
     * Finds the robot's {@link ReplayPolicy} field. Any visibility is accepted; the first non-null value wins.
     * Without a field the default policy is returned.
     *
     * @param robot robot instance
     * @return policy
     */
    static ReplayPolicy discoverPolicy(Object robot) {
        for (Class<?> type = robot.getClass(); type != null && type != Object.class; type = type.getSuperclass()) {
            for (Field field : type.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) || field.getType() != ReplayPolicy.class) {
                    continue;
                }
                try {
                    field.setAccessible(true);
                    Object value = field.get(robot);
                    if (value instanceof ReplayPolicy policy) {
                        return policy;
                    }
                } catch (IllegalAccessException | RuntimeException exception) {
                    throw new IllegalStateException("Cannot read replay policy field " + field + ".", exception);
                }
            }
        }
        return ReplayPolicy.of();
    }

    static Path defaultDirectory(boolean simulation) {
        return simulation
                ? Path.of(System.getProperty("java.io.tmpdir"), "athena", "replays")
                : ROBOT_REPLAY_DIRECTORY;
    }

    ReplayRecorder recorder() {
        return recorder;
    }

    /** Called once after the first robot-init cycle. */
    void boot() {
        recorder.mark(ReplayMarkers.BOOT, "robot code started");
    }

    /** Called after each lifecycle init cycle. */
    void initialized(LifecycleMode mode) {
        switch (mode) {
            case AUTONOMOUS -> recorder.enabledPeriod(ReplayMarkers.AUTO_START, "autonomous started");
            case TELEOP -> recorder.enabledPeriod(ReplayMarkers.TELEOP_START, "teleop started");
            case TEST -> recorder.enabledPeriod(ReplayMarkers.TEST_START, "test started");
            case DISABLED -> recorder.disabled();
            default -> {
            }
        }
    }

    /** Called after each lifecycle exit cycle. Teleop ending is the match end. */
    void exited(LifecycleMode mode) {
        if (mode == LifecycleMode.TELEOP) {
            recorder.matchEnd();
        }
    }

    /** Called every robot cycle: brownout recovery and match identification. */
    void periodic() {
        boolean nowBrowned = RobotController.isBrownedOut();
        if (browned && !nowBrowned) {
            recorder.brownoutRecovered();
        }
        browned = nowBrowned;
        ReplayMatch current = match.get();
        if (current != null && current.identified() && !current.equals(lastMatch)) {
            lastMatch = current;
            recorder.matchIdentified(current);
        }
    }

    private static ReplayMatch driverStationMatch() {
        return new ReplayMatch(
                DriverStation.getEventName(),
                DriverStation.getMatchType().name(),
                DriverStation.getMatchNumber());
    }
}
