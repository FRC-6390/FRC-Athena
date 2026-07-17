package frc.robot.examples;

import ca.frc6390.athena.robot.RuntimeWorker;
import ca.frc6390.athena.robot.RuntimeWorkers;

/** Optional periodic work that can be attached to RobotRuntime.workers(...). */
public final class RuntimeWorkerExamples {
    private RuntimeWorkerExamples() {
    }

    public static RuntimeWorkers inline(Runnable sampleSensors, Runnable updateDiagnostics) {
        return RuntimeWorkers.inline(
                RuntimeWorker.every("fast-signals", 0.005, sampleSensors),
                RuntimeWorker.every("diagnostics", 0.100, updateDiagnostics))
                .onFailure(failure -> System.err.println(
                        failure.worker().name() + ": " + failure.exception().getMessage()));
    }
}
