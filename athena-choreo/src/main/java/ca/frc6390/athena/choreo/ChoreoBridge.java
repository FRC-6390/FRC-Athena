package ca.frc6390.athena.choreo;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Minimal Choreo bridge for the new vendordep system.
 */
public final class ChoreoBridge {
    private ChoreoBridge() {
    }

    public static <ST extends TrajectorySample<ST>> AutoFactory createFactory(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> poseReset,
            Consumer<ST> sampleConsumer,
            boolean mirrorForAlliance,
            Subsystem drivetrain) {
        return new AutoFactory(poseSupplier, poseReset, sampleConsumer, mirrorForAlliance, drivetrain);
    }

    public static <ST extends TrajectorySample<ST>> Optional<Trajectory<ST>> loadTrajectory(String name) {
        return Choreo.loadTrajectory(name);
    }

    public static String[] availableTrajectories() {
        return Choreo.availableTrajectories();
    }
}
