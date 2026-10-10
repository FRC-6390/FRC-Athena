package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

/** The mechanism-examples turret, run headless through the robot runtime and the simulation. */
class TurretMotionSimulationTest {
    @Test
    void brakedTurretReleasedAtSpeedStopsWithinAFewDegrees() {
        TurretMechanism turret = new TurretMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(turret);
        simulation.encoder(turret.angle).positionRotations(turret.angle.rotationsFromPosition(0.0))
                .velocityRotationsPerSecond(turret.angle.rotationsFromPosition(180.0));

        runtime.request(turret.neutral);
        for (int cycle = 0; cycle < 50; cycle++) {
            double now = cycle * 0.02;
            runtime.teleopPeriodic(now, 0.02);
            runtime.simulationPeriodic(now, 0.02);
        }

        double travelled = simulation.encoder(turret.angle).positionRotations() * 360.0;
        assertTrue(Math.abs(travelled) < 5.0,
                () -> "a braked turret released at 180 deg/s should stop within a few degrees, travelled "
                        + travelled);
    }

    private static final class TurretMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 16).brake().currentLimit(25);
        private final EncoderDevice angle = motor.encoder().conversion(360.0).units(EncoderUnit.DEGREES);
        private final ImuDevice chassisImu = ImuDevice.of(ImuKinds.PIGEON_2, 17);
        private final ControlBinding<?> fieldHeading = Controls.position(motor)
                .feedback(angle)
                .dependency(chassisImu)
                .loop(ControlLoops.targetTransform(binding -> context ->
                        ControlOutput.position(context.target() - chassisImu.angleDegrees())))
                .pid(0.144, 0.0, 0.006)
                .constraints(Constraints.range(Range.degrees(-270.0, 270.0)), Constraints.motion(180.0, 540.0))
                .planner(MotionPlanners.boundedAngular(360.0));
        private final Action neutral = fieldHeading.neutral();
    }
}
