package ca.frc6390.athena.examples;

import ca.frc6390.athena.auto.AutoChooserSpec;
import ca.frc6390.athena.auto.AutoRegistry;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.vendor.choreo.ChoreoAutoFactoryAdapter;
import ca.frc6390.athena.vendor.choreo.ChoreoAutoSource;
import ca.frc6390.athena.vendor.choreo.ChoreoAutos;
import ca.frc6390.athena.vendor.pathplanner.PathPlannerAutoSource;
import ca.frc6390.athena.vendor.pathplanner.PathPlannerAutos;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Example PathPlanner and Choreo autonomous source registration.
 */
public final class AutoVendorAdapterExample {
    private AutoVendorAdapterExample() {
    }

    /**
     * Creates an autonomous chooser backed by registered path-tool sources.
     *
     * @return chooser spec
     */
    public static AutoChooserSpec chooser() {
        PathPlannerAutos.register(AutoRegistry.get());
        ChoreoAutos.register(AutoRegistry.get());
        return Autos.chooser()
                .routine("pathplannerLeave", routine -> routine
                        .displayName("PathPlanner Leave")
                        .fromSource(PathPlannerAutoSource.KEY, "LeaveCommunity"))
                .routine("choreoScore", routine -> routine
                        .displayName("Choreo Score")
                        .fromSource(ChoreoAutoSource.KEY, "ScorePreload"))
                .defaultRoutine("pathplannerLeave")
                .toSpec();
    }

    /**
     * Creates a real Choreo trajectory command through the Athena Choreo adapter.
     *
     * @param adapter Choreo factory adapter
     * @return WPILib command for the score preload trajectory
     */
    public static Command choreoTrajectoryCommand(ChoreoAutoFactoryAdapter adapter) {
        return adapter.trajectoryCommand("ScorePreload");
    }

    /**
     * Creates a Choreo routine command through the Athena Choreo adapter.
     *
     * @param adapter Choreo factory adapter
     * @return WPILib command for the two-piece routine
     */
    public static Command choreoRoutineCommand(ChoreoAutoFactoryAdapter adapter) {
        return adapter.routineCommand("TwoPiece");
    }
}
