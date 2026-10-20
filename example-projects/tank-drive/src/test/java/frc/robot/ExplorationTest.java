package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.explore.ExplorationReport;
import ca.frc6390.athena.robot.explore.InputModel;
import ca.frc6390.athena.robot.explore.ModeSchedule;
import ca.frc6390.athena.robot.explore.SimExplorer;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Runs the drive robot for two simulated minutes of seeded random modes and random driver input. The drive has no
 * shared resources, so the explorer should report nothing: no starvation, no faults, no exceptions.
 */
class ExplorationTest {
    @TempDir
    Path directory;

    @Test
    void seededRandomPlayHasNoFindings() {
        ExplorationReport report = SimExplorer.of(session -> RobotRuntime.simulated(session).register(new Robot()))
                .seed(7)
                .duration(Seconds.of(120))
                .controllers(0)
                .modes(ModeSchedule.random())
                .inputs(InputModel.standard())
                .save(directory)
                .run();

        assertTrue(report.clean(), report::describe);
    }
}
