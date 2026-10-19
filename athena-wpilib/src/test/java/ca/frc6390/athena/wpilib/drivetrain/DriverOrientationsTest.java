package ca.frc6390.athena.wpilib.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.drivetrain.DriverOrientation;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class DriverOrientationsTest {
    private static final double EPS = 1.0e-9;

    @Test
    void redAllianceFromTheFacadeFlipsForwardAndSeedsTheLocalization() {
        Localization localization = Localizations.filter();
        DriverOrientation orientation = DriverOrientations.forAlliance(
                () -> Optional.of(DriverOrientation.Alliance.RED), () -> localization);
        localization.reset(new PoseSnapshot(1.0, 2.0, 0.25)).apply(null);

        assertTrue(orientation.allianceKnown());
        assertEquals(Math.PI, orientation.forwardRadians(), EPS);
        assertTrue(localization.seeded(), "the reset through the facade seeds the localization");
    }

    @Test
    void driverStationFacadeReportsAnUnknownAllianceWithoutADriverStation() {
        DriverOrientation orientation = DriverOrientations.forDriverStation(() -> Localizations.filter());

        assertFalse(orientation.allianceKnown());
        assertEquals(0.0, orientation.forwardRadians(), EPS);
    }

    @Test
    void localizationBoundAfterConstructionIsResolvedOnUse() {
        Holder holder = new Holder();
        DriverOrientation orientation = DriverOrientations.forDriverStation(() -> holder.localization);

        assertThrows(IllegalStateException.class, orientation::driverHeadingRadians);
        holder.localization = Localizations.filter();
        assertEquals(0.0, orientation.driverHeadingRadians(), EPS);
    }

    @Test
    void teleopEnableSeedsAnUnseededLocalizationThroughTheDiscoveredHook() {
        Holder holder = new Holder();
        Localization localization = Localizations.filter();
        holder.localization = localization;
        DriveMechanism drive = new DriveMechanism(DriverOrientations.forAlliance(
                () -> Optional.of(DriverOrientation.Alliance.RED), () -> holder.localization));
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(drive);
        assertFalse(localization.seeded());

        runtime.periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, false, false),
                new EventContext(0.0, 0.02, LifecycleMode.TELEOP, LifecyclePhase.INIT, true, false));

        assertTrue(localization.seeded(), "the teleop-start hook reset the unseeded localization");
    }

    private static final class Holder {
        Localization localization;
    }

    /** A robot mechanism that declares only the orientation; its teleop-start hook is discovered from the field. */
    private static final class DriveMechanism implements Mechanism {
        private final DriverOrientation orientation;

        private DriveMechanism(DriverOrientation orientation) {
            this.orientation = orientation;
        }
    }
}
