package ca.frc6390.athena.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class RobotDriveCommandsTest {
    @Test
    void tankDriveWritesDifferentialVelocityAndStopsOnEnd() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);
        CommandRunner runner = new CommandRunner(RobotDriveCommands.tankDrive(speeds, () -> 1.0, () -> 3.0, 0.5));

        runner.step();

        assertEquals(2.0, speeds.calculate().xMetersPerSecond(), 1.0e-9);
        assertEquals(4.0, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void arcadeDriveWritesForwardAndAngularVelocity() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);
        CommandRunner runner = new CommandRunner(RobotDriveCommands.arcadeDrive(speeds, () -> 2.5, () -> -0.75));

        runner.step();

        assertEquals(2.5, speeds.calculate().xMetersPerSecond(), 1.0e-9);
        assertEquals(-0.75, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void fieldRelativeDriveStoresFieldRelativeSource() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);
        CommandRunner runner = new CommandRunner(RobotDriveCommands.fieldRelativeDrive(speeds, () -> 2.0, () -> 0.0, () -> 0.25));

        runner.step();

        assertEquals(0.0, speeds.calculate(Math.PI / 2.0).xMetersPerSecond(), 1.0e-9);
        assertEquals(-2.0, speeds.calculate(Math.PI / 2.0).yMetersPerSecond(), 1.0e-9);
        assertEquals(0.25, speeds.calculate(Math.PI / 2.0).angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void driveDistanceStopsWhenMeasuredDistanceReachesTarget() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);
        AtomicReference<Double> measured = new AtomicReference<>(0.0);
        CommandRunner runner = new CommandRunner(RobotDriveCommands.driveDistance(speeds, measured::get, 2.0, 1.5));

        assertEquals(false, runner.step());
        assertEquals(1.5, speeds.calculate().xMetersPerSecond(), 1.0e-9);

        measured.set(2.0);
        assertTrue(runner.step());
        assertEquals(0.0, speeds.calculate().xMetersPerSecond(), 1.0e-9);
    }

    @Test
    void visionTurnAssistWritesFeedbackUntilAligned() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);
        AtomicReference<Double> yaw = new AtomicReference<>(0.25);
        CommandRunner runner = new CommandRunner(RobotDriveCommands.visionTurnAssist(speeds, () -> true, yaw::get, -2.0, 0.05));

        assertEquals(false, runner.step());
        assertEquals(-0.5, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);

        yaw.set(0.04);
        assertTrue(runner.step());
        assertEquals(0.0, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void followWaypointsWritesFieldRelativeSpeedsUntilPathEnds() {
        RobotSpeeds speeds = new RobotSpeeds(10.0, 10.0);
        AtomicReference<PoseSnapshot> pose = new AtomicReference<>(new PoseSnapshot(0.0, 0.0, 0.0));
        CommandRunner runner = new CommandRunner(RobotDriveCommands.followWaypoints(
                speeds,
                pose::get,
                List.of(new PoseSnapshot(2.0, 0.0, Math.PI / 2.0), new PoseSnapshot(2.0, 1.0, Math.PI / 2.0)),
                1.5,
                2.0,
                0.1));

        assertEquals(false, runner.step());
        assertEquals(1.5, speeds.calculate().xMetersPerSecond(), 1.0e-9);
        assertEquals(0.0, speeds.calculate().yMetersPerSecond(), 1.0e-9);
        assertEquals(Math.PI, speeds.calculate().angularRadiansPerSecond(), 1.0e-9);

        pose.set(new PoseSnapshot(2.0, 0.0, Math.PI / 2.0));
        assertEquals(false, runner.step());
        assertEquals(0.0, speeds.calculate().xMetersPerSecond(), 1.0e-9);

        assertEquals(false, runner.step());
        assertEquals(1.0, speeds.calculate().yMetersPerSecond(), 1.0e-9);

        pose.set(new PoseSnapshot(2.0, 1.0, Math.PI / 2.0));
        assertTrue(runner.step());
        assertEquals(0.0, speeds.calculate().xMetersPerSecond(), 1.0e-9);
        assertEquals(0.0, speeds.calculate().yMetersPerSecond(), 1.0e-9);
    }
}
