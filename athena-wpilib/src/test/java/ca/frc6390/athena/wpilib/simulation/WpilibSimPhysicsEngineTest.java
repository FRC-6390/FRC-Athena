package ca.frc6390.athena.wpilib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleTarget;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import edu.wpi.first.units.Units;

class WpilibSimPhysicsEngineTest {
    @Test
    void gearedMotorSimulationPublishesRotorUnitsToItsIntegratedEncoder() {
        double reduction = 5.27;
        double wheelDiameterMeters = 0.1016;
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 101);
        EncoderDevice driveDistance = motor.encoder()
                .reduction(reduction)
                .measures(Units.Meters)
                .travelPerRevolution(Units.Meters.of(Math.PI * wheelDiameterMeters));
        SimulationSession session = sessionWith(
                "geared-drive",
                SimModel.motor(motor)
                        .encoder(driveDistance)
                        .gearRatio(GearRatio.reduction(reduction, 1.0)));

        session.hardwareGraph().motor(motor).setVoltage(12.0);
        session.step(Units.Seconds.of(1.0));

        double wheelMetersPerSecond = driveDistance.velocityFromRotationsPerSecond(
                session.encoder(driveDistance).velocityRotationsPerSecond());
        assertTrue(wheelMetersPerSecond > 4.0,
                "MK5N R3 wheel speed was incorrectly reduced twice: " + wheelMetersPerSecond);
    }

    @Test
    void motorModelPublishesWpilibPhysicsBackToAthenaHandles() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 1);
        WpilibSimPhysicsEngine engine = new WpilibSimPhysicsEngine();
        SimulationSession session = sessionWith(engine, "motor", SimModel.motor(motor).encoder(encoder));

        session.hardwareGraph().motor(motor).setVoltage(12.0);
        session.step(Units.Seconds.of(0.2));

        assertTrue(session.motor(motor).integratedPositionRotations() > 0.0);
        assertTrue(session.encoder(encoder).positionRotations() > 0.0);
        assertTrue(engine.estimatedBatteryVoltage() > 0.0);
        assertTrue(engine.estimatedBatteryVoltage() <= 12.0);
    }

    @Test
    void everyBuiltInPhysicalMotorHasWpilibPhysicsConstants() {
        int id = 100;
        for (MotorKinds kind : MotorKinds.values()) {
            MotorDevice motor = MotorDevice.of(kind, id++);
            SimulationSession session = sessionWith(kind.name(), SimModel.motor(motor));
            session.hardwareGraph().motor(motor).setVoltage(6.0);
            session.step(Units.Seconds.of(0.02));
            assertTrue(Double.isFinite(session.motor(motor).integratedVelocityRotationsPerSecond()));
        }
    }

    @Test
    void batteryVoltageSinkReceivesEstimatedLoadedVoltage() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 5);
        AtomicReference<Double> voltage = new AtomicReference<>();
        WpilibSimPhysicsEngine engine = new WpilibSimPhysicsEngine(voltage::set);
        SimulationSession session = sessionWith(engine, "motor", SimModel.motor(motor));

        session.hardwareGraph().motor(motor).setVoltage(12.0);
        session.step(Units.Seconds.of(0.2));

        assertTrue(voltage.get() > 0.0);
        assertTrue(voltage.get() <= 12.0);
    }

    @Test
    void flywheelModelPublishesVelocityBackToAthenaHandles() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 2);
        EncoderDevice encoder = HardwareBus.rio()
                .dio(2).encoder(EncoderKinds.REV_THROUGH_BORE);
        SimulationSession session = sessionWith("flywheel", SimModel.flywheel(motor)
                .encoder(encoder)
                .momentOfInertia(Units.KilogramSquareMeters.of(0.004)));

        session.hardwareGraph().motor(motor).setPercentOutput(1.0);
        session.step(Units.Seconds.of(0.2));

        assertTrue(session.motor(motor).integratedVelocityRotationsPerSecond() > 0.0);
        assertTrue(session.encoder(encoder).velocityRotationsPerSecond() > 0.0);
    }

    @Test
    void rootRuntimeMechanismActionDrivesWpilibFlywheelSimulation() {
        FlywheelMechanism mechanism = new FlywheelMechanism();
        SimulationSession session = SimulationSession.create()
                .physicsEngine(new WpilibSimPhysicsEngine());
        RobotRuntime runtime = RobotRuntime.simulated(session).register(mechanism);

        runtime.request(mechanism.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 0.2);

        assertEquals(1, session.registeredModels().size());
        assertTrue(session.motor(mechanism.motor).integratedVelocityRotationsPerSecond() > 0.0);
        assertTrue(session.encoder(mechanism.encoder).velocityRotationsPerSecond() > 0.0);
    }

    @Test
    void armModelUsesWpilibArmSimulationAndRange() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 3);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 3);
        SimModel model = SimModel.arm(motor)
                .encoder(encoder)
                .range(Range.of(Units.Rotations.of(-0.25), Units.Rotations.of(0.25)))
                .length(edu.wpi.first.units.Units.Meters.of(0.45))
                .momentOfInertia(Units.KilogramSquareMeters.of(0.02));
        SimulationSession session = sessionWith("arm", model);

        session.hardwareGraph().motor(motor).setVoltage(12.0);
        session.step(Units.Seconds.of(0.2));

        assertTrue(session.encoder(encoder).positionRotations() > 0.0);
        assertTrue(session.encoder(encoder).positionRotations() <= 0.25);
    }

    @Test
    void elevatorModelUsesWpilibElevatorSimulationAndRange() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO_VORTEX, 4);
        EncoderDevice encoder = HardwareBus.rio()
                .dio(4).encoder(EncoderKinds.REV_THROUGH_BORE);
        SimulationSession session = sessionWith("elevator", SimModel.elevator(motor)
                .encoder(encoder)
                .range(Range.of(Units.Value.of(0.0), Units.Value.of(1.0))));

        session.hardwareGraph().motor(motor).setVoltage(12.0);
        session.step(Units.Seconds.of(0.2));

        assertTrue(session.encoder(encoder).positionRotations() > 0.0);
        assertTrue(session.encoder(encoder).positionRotations() <= 1.0);
    }

    @Test
    void swerveDriveDescriptorIntegratesSimPoseFromModuleTargets() {
        SwerveModule frontLeft = module(1, 2, 11);
        SwerveModule frontRight = module(3, 4, 12);
        SwerveModule backLeft = module(5, 6, 13);
        SwerveModule backRight = module(7, 8, 14);
        SimulationSession session = swerveSession(frontLeft, frontRight, backLeft, backRight);

        setModule(frontLeft, session, 1.0, 0.0);
        setModule(frontRight, session, 1.0, 0.0);
        setModule(backLeft, session, 1.0, 0.0);
        setModule(backRight, session, 1.0, 0.0);
        session.step(Units.Seconds.of(1.0));

        assertEquals(1.0, session.pose().xMeters(), 1.0e-9);
        assertEquals(0.0, session.pose().yMeters(), 1.0e-9);
        assertEquals(0.0, session.pose().headingRadians(), 1.0e-9);
    }

    @Test
    void swerveDriveDescriptorIntegratesHeadingAndSyncsImuYaw() {
        SwerveModule frontLeft = module(21, 22, 31);
        SwerveModule frontRight = module(23, 24, 32);
        SwerveModule backLeft = module(25, 26, 33);
        SwerveModule backRight = module(27, 28, 34);
        SimulationSession session = swerveSession(frontLeft, frontRight, backLeft, backRight);
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 60);
        session.imu(imu);
        double moduleSpeed = Math.hypot(0.25, 0.25);

        setModule(frontLeft, session, moduleSpeed, 0.375);
        setModule(frontRight, session, moduleSpeed, 0.125);
        setModule(backLeft, session, moduleSpeed, 0.625);
        setModule(backRight, session, moduleSpeed, -0.125);
        session.step(Units.Seconds.of(1.0));

        assertEquals(0.0, session.pose().xMeters(), 1.0e-9);
        assertEquals(0.0, session.pose().yMeters(), 1.0e-9);
        assertEquals(1.0, session.pose().headingRadians(), 1.0e-9);
        assertEquals(Math.toDegrees(1.0), session.imu(imu).yawDegrees(), 1.0e-9);
    }

    @Test
    void rootRuntimeSwerveMechanismDrivesWpilibPoseSimulationFromModelList() {
        SwerveDriveMechanism drive = new SwerveDriveMechanism();
        SimulationSession session = SimulationSession.create()
                .physicsEngine(new WpilibSimPhysicsEngine());
        RobotRuntime runtime = RobotRuntime.simulated(session).register(drive);

        runtime.request(drive.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 1.0);

        assertEquals(1, session.registeredModels().size());
        assertTrue(session.pose().xMeters() > 0.0);
        assertEquals(0.0, session.pose().yMeters(), 1.0e-9);
    }

    @Test
    void rootRuntimeLocalizationSeesSameTickWpilibSimulationPose() {
        SwerveDriveMechanism drive = new SwerveDriveMechanism();
        SimulationSession session = SimulationSession.create()
                .physicsEngine(new WpilibSimPhysicsEngine());
        Localization localization = Localizations.latestValid()
                .input(Measurements.poses(() -> new Pose2d(session.pose().xMeters(), session.pose().yMeters(), new Rotation2d(session.pose().headingRadians()))));
        RobotRuntime runtime = RobotRuntime.simulated(session)
                .register(drive)
                .localization(localization);

        runtime.request(drive.initial);
        runtime.robotPeriodic(0.0, 0.02);
        runtime.simulationPeriodic(0.02, 1.0);

        assertTrue(session.pose().xMeters() > 0.0);
        assertEquals(session.pose().xMeters(), localization.pose().getX(), 1.0e-9);
        assertEquals(session.pose().yMeters(), localization.pose().getY(), 1.0e-9);
        assertEquals(session.pose().headingRadians(), localization.pose().getRotation().getRadians(), 1.0e-9);
    }

    private static SimulationSession sessionWith(String name, SimModel model) {
        return sessionWith(new WpilibSimPhysicsEngine(), name, model);
    }

    private static SimulationSession sessionWith(WpilibSimPhysicsEngine engine, String name, SimModel model) {
        return SimulationSession.create()
                .physicsEngine(engine)
                .model(name, model);
    }

    private static SimulationSession swerveSession(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        SwerveKinematics kinematics = SwerveKinematics.rectangular(
                Meters.of(0.5), Meters.of(0.5), MetersPerSecond.of(4.0), frontLeft, frontRight, backLeft, backRight);
        return SimulationSession.create()
                .physicsEngine(new WpilibSimPhysicsEngine())
                .model("swerve", kinematics.simulationModel());
    }

    private static void setModule(SwerveModule module, SimulationSession session, double speedMetersPerSecond, double angleRotations) {
        session.hardwareGraph().motor(module.drive.get()).setVelocityTargetRotationsPerSecond(speedMetersPerSecond);
        session.hardwareGraph().motor(module.steer.get()).setPositionTargetRotations(angleRotations);
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

    private static final class FlywheelMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 70);
        private final EncoderDevice encoder = HardwareBus.rio()
                .dio(7).encoder(EncoderKinds.REV_THROUGH_BORE);
        @SuppressWarnings("unused")
        private final SimModel simulation = SimModel.flywheel(motor)
                .encoder(encoder)
                .momentOfInertia(Units.KilogramSquareMeters.of(0.004));
        @SuppressWarnings("unused")
        private final Action initial = motor.percent(1.0);
    }

    private static final class SwerveDriveMechanism implements ca.frc6390.athena.mechanism.core.Mechanism {
        private final SwerveModule frontLeft = module(81, 82, 91);
        private final SwerveModule frontRight = module(83, 84, 92);
        private final SwerveModule backLeft = module(85, 86, 93);
        private final SwerveModule backRight = module(87, 88, 94);
        @SuppressWarnings("unused")
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                Meters.of(0.5),
                Meters.of(0.5),
                MetersPerSecond.of(4.0),
                frontLeft,
                frontRight,
                backLeft,
                backRight);
        @SuppressWarnings("unused")
        private final Action initial = Actions.parallel(
                frontLeft.target(new SwerveModuleTarget(1.0, 0.0)),
                frontRight.target(new SwerveModuleTarget(1.0, 0.0)),
                backLeft.target(new SwerveModuleTarget(1.0, 0.0)),
                backRight.target(new SwerveModuleTarget(1.0, 0.0)));
    }
}
