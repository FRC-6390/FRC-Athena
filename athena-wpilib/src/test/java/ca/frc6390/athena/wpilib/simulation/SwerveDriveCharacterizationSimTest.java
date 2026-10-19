package ca.frc6390.athena.wpilib.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveDriveCharacterization;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

/** Runs the whole-drivetrain drive characterization against the swerve simulation plant. */
class SwerveDriveCharacterizationSimTest {
    private static final double DT_SECONDS = 0.02;
    private static final double TEST_VOLTS = 6.0;

    @Test
    void driveCharacterizationFitsKvCloseToSimulatedPlantSteadyState() {
        DriveUnderTest robot = new DriveUnderTest();
        SimulationSession session = SimulationSession.create().physicsEngine(new WpilibSimPhysicsEngine());
        RobotRuntime runtime = RobotRuntime.simulated(session).register(robot);

        runtime.request(robot.characterization.quasistaticForward());
        double now = 0.0;
        for (int cycle = 0; cycle < 450; cycle++) {
            runtime.robotPeriodic(now, DT_SECONDS);
            runtime.simulationPeriodic(now, DT_SECONDS);
            now += DT_SECONDS;
        }

        var samples = robot.characterization.samples();
        assertFalse(samples.isEmpty(), "characterization recorded no samples");
        assertTrue(samples.stream().allMatch(sample -> sample.voltage() >= 0.0 && sample.voltage() <= TEST_VOLTS),
                "quasistatic ramp must stay between zero and the test voltage");
        assertEquals(TEST_VOLTS, samples.get(samples.size() - 1).voltage(), 1.0e-9,
                "the ramp must hold the test voltage once it reaches the limit");

        // Steady state at the voltage limit: average measured wheel speed over the final second.
        double steadySum = 0.0;
        int steadyCount = 0;
        for (int index = samples.size() - 50; index < samples.size(); index++) {
            steadySum += samples.get(index).velocity();
            steadyCount++;
        }
        double steadyVelocity = steadySum / steadyCount;
        assertTrue(steadyVelocity > 0.5, "drive did not spin up: " + steadyVelocity + " m/s");
        double steadyStateKv = TEST_VOLTS / steadyVelocity;

        double fittedKv = robot.characterization.kV();
        assertTrue(Double.isFinite(fittedKv) && fittedKv > 0.0, "kV must be positive, got " + fittedKv);
        assertEquals(steadyStateKv, fittedKv, 0.05 * steadyStateKv,
                "fitted kV should match the simulated plant's volts per meter-per-second");
        assertEquals(0.0, robot.characterization.kA(), 0.5 * fittedKv,
                "kA should be small relative to kV on a steady ramp");
        assertTrue(Double.isFinite(robot.characterization.kS().in(Volts)));
    }

    /** Four-module drivetrain with its characterization declared as a field, the way robot code does it. */
    private static final class DriveUnderTest implements Mechanism {
        private final SwerveModule frontLeft = module(1, 2, 11);
        private final SwerveModule frontRight = module(3, 4, 12);
        private final SwerveModule backLeft = module(5, 6, 13);
        private final SwerveModule backRight = module(7, 8, 14);
        @SuppressWarnings("unused")
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                Meters.of(0.5), Meters.of(0.5), MetersPerSecond.of(4.0),
                frontLeft, frontRight, backLeft, backRight);
        private final SwerveDriveCharacterization characterization = kinematics.characterization()
                .maxVoltage(Volts.of(TEST_VOLTS))
                .rampRate(Volts.of(1.0).per(Seconds))
                .timeout(Seconds.of(8.0));
    }

    private static SwerveModule module(int driveMotorId, int steerMotorId, int encoderId) {
        HardwareBus canivore = HardwareBus.can("canivore");
        return new SwerveModules.SDS.MK5N.R3()
                .drive.fill(canivore.motor(MotorKinds.KRAKEN_X60, driveMotorId))
                .steer.fill(canivore.motor(MotorKinds.KRAKEN_X44, steerMotorId))
                .angle.fill(canivore.encoder(EncoderKinds.CANCODER, encoderId))
                .driveMaxSpeed(MetersPerSecond.of(4.0))
                .steerPid(1.9, 0.0, 0.0);
    }
}
