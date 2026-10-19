package ca.frc6390.athena.wpilib.simulation;

import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.model.FrictionTable;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * A simulated arm whose physics is the model it was configured with: friction, gravity, kV and kA all come
 * from the {@link MechanismModel}, so the characterization and feedforward see the mechanism they describe.
 */
class ModelPlantSimulationTest {
    private static final double DT = 0.02;
    private static final double STATIC_FRICTION = 0.5;
    private static final double KINETIC_FRICTION = 0.4;
    private static final double KV = 2.0;
    private static final double KA = 0.2;
    private static final double KG = 1.0;

    /** Arm with the configured parameters, using the KrakenX60 constants the characterization also uses. */
    private static MechanismModel armModel(double kG) {
        DCMotor motor = DCMotor.getKrakenX60(1);
        return new MechanismModel(MotionKind.ROTARY, STATIC_FRICTION, KINETIC_FRICTION, KV, KA, kG, 0.0,
                FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, motor.KtNMPerAmp, motor.rOhms, 1,
                List.of(MotorKinds.KRAKEN_X60.key()), 1.0, Map.of(), 1.0, 0);
    }

    private static SimulationSession armSession(MotorDevice motor, EncoderDevice encoder, MechanismModel model) {
        return SimulationSession.create()
                .physicsEngine(new WpilibSimPhysicsEngine())
                .model("arm", SimModel.arm(motor)
                        .encoder(encoder)
                        .range(Range.rotations(-50.0, 50.0))
                        .model(model));
    }

    @Test
    void feedforwardOnlyHoldsTheArmAgainstGravity() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 201);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 201);
        MechanismModel model = armModel(KG);
        SimulationSession session = armSession(motor, encoder, model);

        // The arm starts level, where gravity pulls with KG volts; a feedforward of that size holds it.
        double start = session.encoder(encoder).positionRotations();
        for (int cycle = 0; cycle < 250; cycle++) {
            double volts = model.feedforwardVolts(session.encoder(encoder).positionRotations(), 0.0, 0.0, 0.0);
            session.hardwareGraph().motor(motor).setVoltage(volts);
            session.step(DT);
        }
        assertEquals(start, session.encoder(encoder).positionRotations(), 0.005, "held by feedforward");
    }

    @Test
    void withoutGravityFeedforwardTheArmFallsAsTheModelPredicts() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 202);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 202);
        SimulationSession session = armSession(motor, encoder, armModel(KG));

        session.hardwareGraph().motor(motor).setVoltage(0.0);
        session.step(2.0);

        // Gravity (1 V) exceeds breakaway friction (0.5 V) at the level position, so the arm moves.
        assertTrue(Math.abs(session.encoder(encoder).positionRotations()) > 0.05,
                "arm should fall: " + session.encoder(encoder).positionRotations());
    }

    @Test
    void voltageBelowBreakawayDoesNotMoveTheArmButAboveItDoes() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 203);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 203);
        // No gravity term: only friction opposes the drive, so breakaway is exactly the static friction voltage.
        MechanismModel frictionOnly = armModel(0.0);
        SimulationSession session = armSession(motor, encoder, frictionOnly);

        // 80% of breakaway: friction holds the arm at rest.
        session.hardwareGraph().motor(motor).setVoltage(0.8 * STATIC_FRICTION);
        session.step(3.0);
        assertEquals(0.0, session.encoder(encoder).positionRotations(), 1.0e-9, "below breakaway: no motion");
        assertEquals(0.0, session.encoder(encoder).velocityRotationsPerSecond(), 1.0e-9);

        // 150% of breakaway: the arm breaks away and moves.
        session.hardwareGraph().motor(motor).setVoltage(1.5 * STATIC_FRICTION);
        session.step(1.0);
        assertTrue(session.encoder(encoder).positionRotations() > 0.05,
                "above breakaway the arm moves: " + session.encoder(encoder).positionRotations());
    }

    @Test
    void characterizationOfTheSimulatedArmRecoversTheConfiguredParameters() {
        // Gravity below breakaway friction: the arm can rest at any angle, so the quasistatic ramps hold.
        double gravity = 0.3;
        MechanismModel configured = armModel(gravity);
        double[] clock = {0.0};
        ArmMechanism arm = new ArmMechanism(clock);
        SimulationSession session = SimulationSession.create()
                .physicsEngine(new WpilibSimPhysicsEngine());
        RobotRuntime runtime = RobotRuntime.simulated(session).register(arm);
        // The simulation is the configured arm; the mechanism's characterization only reads and drives it.
        session.model("arm", SimModel.arm(arm.motor).encoder(arm.encoder)
                .range(Range.rotations(-50.0, 50.0))
                .model(configured));

        run(runtime, clock, arm.quasistaticForward, 10.0);
        run(runtime, clock, arm.quasistaticReverse, 10.0);
        run(runtime, clock, arm.dynamicForward, 3.0);
        run(runtime, clock, arm.dynamicReverse, 3.0);

        MechanismModel fitted = arm.characterization.fitRuns();
        assertEquals(KV, fitted.kV(), 0.15, "kV");
        assertEquals(KA, fitted.kA(), 0.05, "kA");
        assertEquals(KINETIC_FRICTION, fitted.kineticFrictionVolts(), 0.06, "kinetic friction");
        assertEquals(STATIC_FRICTION, fitted.staticFrictionVolts(), 0.08, "static friction");
        assertEquals(gravity, fitted.kG(), 0.08, "gravity amplitude");
        assertEquals(configured.inertia(), fitted.inertia(), 0.05 * configured.inertia(), "inertia");
        assertTrue(fitted.rSquared() > 0.95, "r2 " + fitted.rSquared());
    }

    private static void run(RobotRuntime runtime, double[] clock, Action action, double seconds) {
        runtime.request(action);
        int cycles = (int) Math.ceil(seconds / DT);
        for (int cycle = 0; cycle < cycles; cycle++) {
            clock[0] += DT;
            runtime.robotPeriodic(clock[0], DT);
            runtime.simulationPeriodic(clock[0], DT);
            if (!runtime.isActionRunning(action)) {
                break;
            }
        }
        runtime.cancel(action);
        runtime.robotPeriodic(clock[0] + DT, DT);
    }

    private static final class ArmMechanism implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 211);
        final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 211).measures(Units.Rotations);
        final Characterization characterization;
        final Action quasistaticForward;
        final Action quasistaticReverse;
        final Action dynamicForward;
        final Action dynamicReverse;

        /** The test reads samples through the model, so nothing is written to the robot log. */
        private static final SysIdLog NO_EXPORT = new SysIdLog() {
            @Override
            public void record(SysIdSample sample) {
            }

            @Override
            public void end() {
            }
        };

        ArmMechanism(double[] clock) {
            // A rotations-typed binding makes the characterization rotary, so gravity is fitted.
            ControlBinding<?> angle = Controls.position(motor, Units.Rotations).feedback(encoder);
            characterization = angle.characterization()
                    .rampRate(Volts.per(Seconds).of(0.5))
                    .maxVoltage(Volts.of(4.0))
                    .exportTo(NO_EXPORT)   // keep the test off the global WPILib DataLog
                    .clock(() -> clock[0]);
            quasistaticForward = characterization.quasistaticForward();
            quasistaticReverse = characterization.quasistaticReverse();
            dynamicForward = characterization.dynamicForward();
            dynamicReverse = characterization.dynamicReverse();
        }
    }
}
