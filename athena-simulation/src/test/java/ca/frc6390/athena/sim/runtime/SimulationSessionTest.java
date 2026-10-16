package ca.frc6390.athena.sim.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimImuHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

class SimulationSessionTest {
    @Test
    void reusesHandlesByStableHardwareIdentity() {
        SimulationSession session = SimulationSession.create();
        MotorDevice rioMotor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice canivoreMotor = rioMotor.canbus("canivore");
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 2);

        assertSame(session.motor(rioMotor), session.motor(rioMotor));
        assertSame(session.imu(imu), session.imu(imu));
        assertEquals(rioMotor, session.motor(rioMotor).device());
        assertEquals(canivoreMotor, session.motor(canivoreMotor).device());
    }

    @Test
    void motorOutputsClampAndIntegrateDuringStep() {
        SimulationSession session = SimulationSession.create();
        SimMotorHandle motor = session.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 3));

        motor.setPercentOutput(2.0);
        session.step(0.5);
        assertEquals(1.0, motor.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(0.5, motor.integratedPositionRotations(), 1.0e-9);

        motor.setVoltage(-6.0);
        session.step(2.0);
        assertEquals(-0.5, motor.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(-0.5, motor.integratedPositionRotations(), 1.0e-9);

        motor.setPositionTargetRotations(4.0);
        assertEquals(4.0, motor.integratedPositionRotations(), 1.0e-9);
        assertEquals(0.0, motor.integratedVelocityRotationsPerSecond(), 1.0e-9);

        motor.setVelocityTargetRotationsPerSecond(3.0);
        session.step(Double.NaN);
        assertEquals(4.0, motor.integratedPositionRotations(), 1.0e-9);
        session.step(0.25);
        assertEquals(4.75, motor.integratedPositionRotations(), 1.0e-9);
    }

    @Test
    void hardwareFollowerMirrorsLeaderCommandsAndDirection() {
        SimulationSession session = SimulationSession.create();
        MotorDevice leaderDevice = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice followerDevice = MotorDevice.of(MotorKinds.KRAKEN_X60, 2)
                .follow(leaderDevice)
                .inverted();
        SimMotorHandle follower = assertInstanceOf(
                SimMotorHandle.class,
                session.hardwareGraph().motor(followerDevice));
        SimMotorHandle leader = session.motor(leaderDevice);

        leader.setPercentOutput(0.6);

        assertEquals(SimMotorHandle.CommandKind.PERCENT, follower.commandKind());
        assertEquals(-0.6, follower.commandValue(), 1.0e-9);
    }

    @Test
    void nonFiniteTimestepsDoNotReachPhysicsEngine() {
        AtomicInteger calls = new AtomicInteger();
        SimulationSession session = SimulationSession.create()
                .physicsEngine((models, runtime, seconds) -> calls.incrementAndGet())
                .model("drive", SimModel.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 4)));

        session.step(Double.NaN);
        session.step(Double.POSITIVE_INFINITY);
        session.step(Double.NEGATIVE_INFINITY);
        session.step(-0.02);
        session.step(0.02);

        assertEquals(1, calls.get());
    }

    @Test
    void imuYawRateIntegratesDuringStep() {
        SimulationSession session = SimulationSession.create();
        SimImuHandle imu = session.imu(ImuDevice.of(ImuKinds.PIGEON_2, 4))
                .yawDegrees(10.0)
                .yawRateDegreesPerSecond(90.0);

        session.step(0.5);

        assertEquals(55.0, imu.yawDegrees(), 1.0e-9);
        assertEquals(55.0, imu.angleDegrees(), 1.0e-9);

        imu.zeroYaw();
        assertEquals(0.0, imu.yawDegrees(), 1.0e-9);
        session.step(0.5);
        assertEquals(45.0, imu.yawDegrees(), 1.0e-9);

        imu.reset();
        session.step(0.5);
        assertEquals(0.0, imu.yawDegrees(), 1.0e-9);
    }

    @Test
    void registeringModelsMaterializesMotorsAndPreservesModelList() {
        SimulationSession session = SimulationSession.create();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 5);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 11);

        session.model("drive", SimModel.motor(motor).encoder(encoder));

        assertEquals(1, session.registeredModels().size());
        assertSame(session.motor(motor), session.motor(motor));
        assertSame(session.encoder(encoder), session.encoder(encoder));
    }

    @Test
    void hardwareGraphUsesRuntimeBackedSimulationHandles() {
        SimulationSession session = SimulationSession.create();
        HardwareGraph graph = session.hardwareGraph();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 6);
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 7);

        MotorHandle graphMotor = graph.motor(motor);

        assertSame(session.motor(motor), graphMotor);
        assertSame(session.imu(imu), graph.imu(imu));

        graphMotor.setVelocityTargetRotationsPerSecond(2.0);
        session.step(0.5);
        assertEquals(1.0, session.motor(motor).integratedPositionRotations(), 1.0e-9);
        assertSame(graph.encoder(motor.encoder()), graph.encoder(motor.encoder()));
        assertEquals(1.0, graph.encoder(motor.encoder()).positionRotations(), 1.0e-9);
    }

    @Test
    void hardwareGraphResolvesSupportedRealMotorKindsInSimulation() {
        SimulationSession session = SimulationSession.create();
        HardwareGraph graph = session.hardwareGraph();
        List<MotorKind> kinds = new java.util.ArrayList<>(List.of(MotorKinds.values()));
        kinds.add(MotorKinds.NEO.controlledBy(MotorControllerKinds.SPARK_FLEX));
        for (int index = 0; index < kinds.size(); index++) {
            MotorKind kind = kinds.get(index);
            MotorDevice motor = MotorDevice.of(kind, index + 1);
            MotorHandle handle = graph.motor(motor);

            assertSame(session.motor(motor), handle);
            assertInstanceOf(SimMotorHandle.class, handle);
        }
    }

    @Test
    void hardwareGraphResolvesSupportedRealEncoderKindsInSimulation() {
        SimulationSession session = SimulationSession.create();
        HardwareGraph graph = session.hardwareGraph();
        List.of(
                EncoderKinds.CANCODER,
                EncoderKinds.REV_THROUGH_BORE,
                EncoderKinds.INTEGRATED_MOTOR)
                .forEach(kind -> {
                    EncoderDevice encoder = EncoderDevice.of(kind, kind.name().hashCode() & 0x7fff);
                    EncoderHandle handle = graph.encoder(encoder);

                    assertSame(session.encoder(encoder), handle);
                    assertInstanceOf(SimEncoderHandle.class, handle);
                });
    }

    @Test
    void hardwareGraphResolvesSupportedRealImuKindsInSimulation() {
        SimulationSession session = SimulationSession.create();
        HardwareGraph graph = session.hardwareGraph();
        List.of(ImuKinds.PIGEON_2, ImuKinds.NAVX)
                .forEach(kind -> {
                    ImuDevice imu = ImuDevice.of(kind, kind.name().hashCode() & 0x7fff);
                    ImuHandle handle = graph.imu(imu);

                    assertSame(session.imu(imu), handle);
                    assertInstanceOf(SimImuHandle.class, handle);
                });
    }

    @Test
    void hardwareGraphUsesRealKindSimulationEncoderHandles() {
        SimulationSession session = SimulationSession.create();
        HardwareGraph graph = session.hardwareGraph();
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 9).canbus("canivore");

        EncoderHandle graphEncoder = graph.encoder(encoder);

        assertSame(session.encoder(encoder), graphEncoder);
        SimEncoderHandle simEncoder = assertInstanceOf(SimEncoderHandle.class, graphEncoder);
        simEncoder.positionRotations(2.5)
                .absolutePositionRotations(0.25)
                .velocityRotationsPerSecond(12.0);

        assertEquals(2.5, graphEncoder.positionRotations(), 1.0e-9);
        assertEquals(0.25, graphEncoder.absolutePositionRotations(), 1.0e-9);
        assertEquals(12.0, graphEncoder.velocityRotationsPerSecond(), 1.0e-9);
    }

    @Test
    void simulationBackendsAreExplicitAndNotDiscoveredGlobally() {
        BackendRegistry registry = BackendRegistry.discover();

        assertTrue(registry.motorBackendFor(MotorKinds.KRAKEN_X60)
                .map(backend -> !backend.getClass().getName().startsWith("ca.frc6390.athena.sim."))
                .orElse(true));
        assertTrue(registry.encoderBackendFor(EncoderKinds.CANCODER)
                .map(backend -> !backend.getClass().getName().startsWith("ca.frc6390.athena.sim."))
                .orElse(true));
        assertTrue(registry.imuBackendFor(ImuKinds.PIGEON_2)
                .map(backend -> !backend.getClass().getName().startsWith("ca.frc6390.athena.sim."))
                .orElse(true));
    }

    @Test
    void simulationSessionProvidesRealKindHardwareGraphAndStepLoop() {
        SimulationSession session = SimulationSession.create();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 10);
        MotorHandle handle = session.hardwareGraph().motor(motor);

        handle.setVelocityTargetRotationsPerSecond(5.0);
        session.step(0.2);

        assertEquals(1.0, session.motor(motor).integratedPositionRotations(), 1.0e-9);
    }

    @Test
    void simulationSessionIsPublicCoordinatorForNormalRuntimePath() {
        SimulationSession session = SimulationSession.create();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 13);
        MotorHandle handle = session.hardwareGraph().motor(motor);

        handle.setVelocityTargetRotationsPerSecond(2.0);
        session.step(0.25);

        assertSame(session.motor(motor), handle);
        assertEquals(0.5, handle.integratedPositionRotations(), 1.0e-9);
    }

    @Test
    void simulationSessionOwnsPoseResetReadbackAndDriveIntegration() {
        SimulationSession session = SimulationSession.create();
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 11);

        session.imu(imu);
        session.resetPose(new PoseSnapshot(1.0, 2.0, Math.PI / 2.0))
                .drivePose(3.0, -1.0, Math.PI, 0.5);

        assertEquals(2.5, session.pose().xMeters(), 1.0e-9);
        assertEquals(1.5, session.pose().yMeters(), 1.0e-9);
        assertEquals(Math.PI, session.pose().headingRadians(), 1.0e-9);
        assertEquals(180.0, session.imu(imu).yawDegrees(), 1.0e-9);
    }

    @Test
    void updatesVisionSimulationsFromSimPose() {
        SimulationSession session = SimulationSession.create();
        PoseSnapshot[] observedPose = new PoseSnapshot[1];

        session.vision(pose -> observedPose[0] = pose)
                .resetPose(new PoseSnapshot(1.0, 2.0, 0.25))
                .drivePose(3.0, -1.0, 0.5, 0.2)
                .step(0.02);

        assertEquals(1, session.visionSimulations().size());
        assertEquals(1.6, observedPose[0].xMeters(), 1.0e-9);
        assertEquals(1.8, observedPose[0].yMeters(), 1.0e-9);
        assertEquals(0.35, observedPose[0].headingRadians(), 1.0e-9);
    }

    @Test
    void simulationSessionOwnsModelSteppingAndDigitalLimitState() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 12);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 12);
        DigitalInputDevice limit = DigitalInputDevice.rio(0);
        SimulationSession session = SimulationSession.create()
                .model("axis", SimModel.motor(motor).encoder(encoder).limit(limit, 12.0, 1.0));

        MotorHandle handle = session.hardwareGraph().motor(motor);
        handle.setPercentOutput(1.0);
        session.step(0.2);

        assertTrue(session.encoder(encoder).positionRotations() > 0.0);
        assertTrue(session.encoder(encoder).velocityRotationsPerSecond() > 0.0);
        // The X60 with the default 0.01 kg m^2 spins up over about a second, so the 12 rotation limit is
        // reached after a few hundred milliseconds of full output rather than within the first 0.2 s.
        boolean[] active = {false};
        for (int step = 0; step < 500 && !active[0]; step++) {
            session.step(0.02);
            session.withDigitalInputs(() -> active[0] = limit.active());
        }
        assertTrue(active[0], () -> "limit should activate near 12 rotations, reached "
                + session.encoder(encoder).positionRotations());
        assertTrue(session.encoder(encoder).positionRotations() > 11.0);
    }

    @Test
    void defaultModelRunnerPublishesMechanismStateToIntegratedMotorEncoderWithGearing() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 14);
        EncoderDevice mechanismEncoder = EncoderDevice.of(EncoderKinds.CANCODER, 14);
        SimulationSession session = SimulationSession.create().model(
                "geared-axis",
                SimModel.motor(motor)
                        .encoder(mechanismEncoder)
                        .gearRatio(GearRatio.reduction(2.0, 1.0)));
        HardwareGraph graph = session.hardwareGraph();

        graph.motor(motor).setPercentOutput(1.0);
        session.step(0.06);

        // The motor turns twice per mechanism turn: the mechanism encoder reads mechanism rotations and the
        // integrated motor encoder reads motor rotations, whatever the (physical) spin-up speed is.
        double mechanism = session.encoder(mechanismEncoder).positionRotations();
        double mechanismVelocity = session.encoder(mechanismEncoder).velocityRotationsPerSecond();
        assertTrue(mechanism > 0.0 && mechanismVelocity > 0.0);
        assertEquals(2.0 * mechanism, session.motor(motor).integratedPositionRotations(), 1.0e-9);
        assertEquals(2.0 * mechanismVelocity, session.motor(motor).integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(session.motor(motor).integratedPositionRotations(),
                graph.encoder(motor.encoder()).positionRotations(), 1.0e-9);
        assertEquals(session.motor(motor).integratedVelocityRotationsPerSecond(),
                graph.encoder(motor.encoder()).velocityRotationsPerSecond(), 1.0e-9);
    }

    @Test
    void digitalInputStateIsScopedPerSimulationSession() {
        DigitalInputDevice input = DigitalInputDevice.rio(6);
        SimulationSession first = SimulationSession.create();
        SimulationSession second = SimulationSession.create();

        first.digitalInput(input).raw(true);
        second.digitalInput(input).raw(false);

        first.withDigitalInputs(() -> assertTrue(input.active()));
        second.withDigitalInputs(() -> assertEquals(false, input.active()));
    }

    @Test
    void unifiedModelComposesProviderLeavesAndCustomRuntimeRules() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 40);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 40);
        SimModel custom = SimModel.builder()
                .motor(motor)
                .encoder(encoder)
                .runtime(context -> seconds -> {
                    double velocity = context.command(motor).value() * 5.0;
                    double position = context.encoderPosition(encoder) + velocity * seconds;
                    context.motorState(motor, position, velocity);
                    context.encoderState(encoder, position, position, velocity);
                })
                .build();
        SimModel composed = SimModel.motor(MotorDevice.of(MotorKinds.KRAKEN_X44, 41)).and(custom);
        SimulationSession session = SimulationSession.create().model("robot", composed);

        session.hardwareGraph().motor(motor).setPercentOutput(0.5);
        session.step(0.2);

        assertEquals(1, session.registeredModels().size());
        assertEquals(1, composed.physicsLeaves().size());
        assertEquals(0.5, session.encoder(encoder).positionRotations(), 1.0e-9);
        assertEquals(2.5, session.encoder(encoder).velocityRotationsPerSecond(), 1.0e-9);
    }

    @Test
    void brakedMotorReleasedAtSpeedStopsWhileCoastingMotorKeepsRolling() {
        // A 100:1 turret axis with 0.5 kg m^2 of inertia: shorted windings brake it within a couple of degrees.
        double brakedTravel = releasedTravelRotations(MotorDevice.of(MotorKinds.KRAKEN_X44, 50).brake());
        double coastTravel = releasedTravelRotations(MotorDevice.of(MotorKinds.KRAKEN_X44, 51).coast());

        assertTrue(brakedTravel < 0.01,
                () -> "braked axis should stop within about 3.6 degrees at 180 deg/s, travelled "
                        + brakedTravel * 360.0 + " degrees");
        assertTrue(coastTravel > 3.0 * brakedTravel,
                () -> "coasting axis should roll much further: braked=" + brakedTravel
                        + " coast=" + coastTravel);
    }

    /** Releases a 100:1, 0.5 kg m^2 axis at 180 deg/s with no output and returns the rotations it travels. */
    private static double releasedTravelRotations(MotorDevice motor) {
        EncoderDevice encoder = motor.encoder();
        SimulationSession session = SimulationSession.create().model("axis", SimModel.motor(motor)
                .gearRatio(GearRatio.reduction(100.0, 1.0))
                .momentOfInertia(0.5));
        session.encoder(encoder).positionRotations(0.0).velocityRotationsPerSecond(0.5 * 100.0);
        session.motor(motor).stop();

        for (int cycle = 0; cycle < 150; cycle++) {
            session.step(0.02);
        }
        return session.encoder(encoder).positionRotations() / 100.0;
    }

    @Test
    void separateModelsCannotClaimTheSameMotor() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 42);
        SimulationSession session = SimulationSession.create().model("first", SimModel.motor(motor));

        assertThrows(IllegalArgumentException.class, () -> session.model("second", SimModel.motor(motor)));
    }
}
