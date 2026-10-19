package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;
import edu.wpi.first.units.Units;

/** The mechanism-examples turret, run headless through the robot runtime and the simulation. */
class TurretMotionSimulationTest {
    @Test
    void brakedTurretReleasedAtSpeedStopsWithinAFewDegrees() {
        TurretMechanism turret = new TurretMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(turret);
        simulation.encoder(turret.angle).positionRotations(turret.angle.rotationsFromPosition(Units.Degrees.of(0.0)))
                .velocityRotationsPerSecond(turret.angle.rotationsFromPosition(Units.Degrees.of(180.0)));

        runtime.request(turret.neutral);
        for (int cycle = 0; cycle < 50; cycle++) {
            double now = cycle * 0.02;
            runtime.teleopPeriodic(now, 0.02);
            runtime.simulationPeriodic(now, 0.02);
        }

        double travelled = turret.angle.raw().positionFromRotations(simulation.encoder(turret.angle).positionRotations());
        assertTrue(Math.abs(travelled) < 5.0,
                () -> "a braked turret released at 180 deg/s should stop within a few degrees, travelled "
                        + travelled);
    }

    @Test
    void gearedTurretSlewsToNinetyDegreesWithoutBangBang() {
        TurretMechanism turret = new TurretMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(turret);
        runtime.request(turret.fieldHeading.position(90.0));

        int signChanges = 0;
        double lastSign = 0.0;
        double firstWithinHalfDegree = -1.0;
        for (int cycle = 0; cycle < 150; cycle++) {
            double now = cycle * 0.02;
            runtime.teleopPeriodic(now, 0.02);
            runtime.simulationPeriodic(now, 0.02);

            SimMotorHandle motor = simulation.motor(turret.motor);
            if (motor.commandKind() == SimMotorHandle.CommandKind.VOLTAGE && Math.abs(motor.commandValue()) > 1.0e-9) {
                double sign = Math.signum(motor.commandValue());
                if (lastSign != 0.0 && sign != lastSign) {
                    signChanges++;
                }
                lastSign = sign;
            }
            double degrees = turret.angle.raw().positionFromRotations(simulation.encoder(turret.angle).positionRotations());
            if (firstWithinHalfDegree < 0.0 && degrees >= 89.5) {
                firstWithinHalfDegree = now;
            }
        }

        double finalDegrees = turret.angle.raw().positionFromRotations(simulation.encoder(turret.angle).positionRotations());
        int changes = signChanges;
        assertTrue(changes <= 2, () -> "the geared turret's voltage should not bang-bang, sign changes " + changes);
        double reached = firstWithinHalfDegree;
        assertTrue(reached > 0.0 && reached < 2.0,
                () -> "a 90 degree move should be within half a degree inside 2 s, reached it at " + reached);
        assertEquals(90.0, finalDegrees, 1.0, "the turret should settle at 90 degrees");
    }

    private static final class TurretMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 16).brake().currentLimit(Units.Amps.of(25));
        private final EncoderDevice angle = motor.encoder()
                .gearing(GearRatio.reduction(100.0, 1.0))
                .measures(Units.Degrees);
        private final SimModel simulation = SimModel.motor(motor)
                .gearRatio(GearRatio.reduction(100.0, 1.0))
                .momentOfInertia(Units.KilogramSquareMeters.of(0.5))
                .range(Range.of(Units.Rotations.of(-0.75), Units.Rotations.of(0.75)));
        private final ImuDevice chassisImu = ImuDevice.of(ImuKinds.PIGEON_2, 17);
        private final ControlBinding<?> fieldHeading = Controls.position(motor)
                .feedback(angle)
                .dependency(chassisImu)
                .loop(ControlLoops.targetTransform(binding -> context ->
                        ControlOutput.position(context.target() - chassisImu.raw().angleDegrees())))
                .pid(0.144, 0.0, 0.006)
                .constraints(Constraints.range(Range.of(Units.Degrees.of(-270.0), Units.Degrees.of(270.0))), Constraints.motion(180.0, 540.0))
                .planner(MotionPlanners.boundedAngular(360.0));
        private final Action neutral = fieldHeading.neutral();
    }
}
