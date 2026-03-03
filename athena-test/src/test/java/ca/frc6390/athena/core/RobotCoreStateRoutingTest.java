package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.localization.RobotDrivetrainLocalizationFactory;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.RegisterableMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanismFactory;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureConfig;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.core.MotionLimits;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

final class RobotCoreStateRoutingTest {

    private enum DirectMechanismState implements SetpointProvider<Double> {
        IDLE(0.0),
        RUN(1.0);

        private final double setpoint;

        DirectMechanismState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    private enum ArmState implements SetpointProvider<Double> {
        RETRACTED(0.0),
        PREP(1.0),
        EXTENDED(2.0);

        private final double setpoint;

        ArmState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    private enum WristState implements SetpointProvider<Double> {
        FOLDED(0.0),
        LEVEL(1.0),
        DEPLOYED(2.0);

        private final double setpoint;

        WristState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    private record ArmSuperSetpoint(ArmState arm, WristState wrist) {
    }

    private enum ArmSuperState implements SetpointProvider<ArmSuperSetpoint> {
        STOW(new ArmSuperSetpoint(ArmState.RETRACTED, WristState.FOLDED)),
        READY(new ArmSuperSetpoint(ArmState.PREP, WristState.LEVEL));

        private final ArmSuperSetpoint setpoint;

        ArmSuperState(ArmSuperSetpoint setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public ArmSuperSetpoint getSetpoint() {
            return setpoint;
        }
    }

    private enum SharedMechanismState implements SetpointProvider<Double> {
        DEFAULT_A(0.0),
        DEFAULT_B(1.0),
        NON_DEFAULT(2.0);

        private final double setpoint;

        SharedMechanismState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    @Test
    void stateSectionRoutesToDirectMechanismAndSuperstructureChildren() {
        StatefulMechanism<DirectMechanismState> direct = mechanism(DirectMechanismState.IDLE);
        StatefulMechanism<SharedMechanismState> sharedOne = mechanism("sharedOne", SharedMechanismState.DEFAULT_A);
        StatefulMechanism<SharedMechanismState> sharedTwo = mechanism("sharedTwo", SharedMechanismState.DEFAULT_B);
        StatefulMechanism<ArmState> arm = mechanism(ArmState.RETRACTED);
        StatefulMechanism<WristState> wrist = mechanism(WristState.FOLDED);
        SuperstructureMechanism<ArmSuperState, ArmSuperSetpoint> superStructure = SuperstructureConfig
                .create("armSuper", ArmSuperState.STOW)
                .mechanisms(mechanisms -> mechanisms
                        .existing(arm, ArmSuperSetpoint::arm)
                        .existing(wrist, ArmSuperSetpoint::wrist))
                .build();

        RobotCore<FakeDrivetrain> core = new RobotCore<>(
                RobotCoreConfig
                .create()
                .drivetrain(__ -> new FakeDriveConfig())
                .mechanisms(mechanisms -> mechanisms
                        .mechanism(direct)
                        .mechanism(sharedOne)
                        .mechanism(sharedTwo)
                        .superstructure(superStructure))
                .build());

        assertEquals(DirectMechanismState.IDLE, direct.stateMachine().goal());
        assertEquals(ArmSuperState.STOW, superStructure.stateMachine().goal());
        assertEquals(SharedMechanismState.DEFAULT_A, sharedOne.stateMachine().goal());
        assertEquals(SharedMechanismState.DEFAULT_B, sharedTwo.stateMachine().goal());

        assertTrue(core.state().force(DirectMechanismState.RUN));
        direct.periodic();
        assertEquals(DirectMechanismState.RUN, direct.stateMachine().goal());

        assertFalse(core.state().force(SharedMechanismState.NON_DEFAULT));
        assertEquals(ArmSuperState.STOW, superStructure.stateMachine().goal());

        assertTrue(core.state().force(ArmSuperState.READY));
        for (int i = 0; i < 5; i++) {
            superStructure.periodic();
            arm.periodic();
            wrist.periodic();
        }
        assertEquals(ArmSuperState.READY, superStructure.stateMachine().goal());
        assertEquals(ArmState.PREP, arm.stateMachine().goal());
        assertEquals(WristState.LEVEL, wrist.stateMachine().goal());

        assertTrue(core.state().force(SharedMechanismState.DEFAULT_A));
        sharedOne.periodic();
        assertEquals(SharedMechanismState.DEFAULT_A, sharedOne.stateMachine().goal());

        assertTrue(core.state().force(SharedMechanismState.DEFAULT_B));
        sharedTwo.periodic();
        assertEquals(SharedMechanismState.DEFAULT_B, sharedTwo.stateMachine().goal());

        assertFalse(core.state().force(SharedMechanismState.NON_DEFAULT));

        assertTrue(core.state().force(WristState.DEPLOYED));
        wrist.periodic();
        assertEquals(WristState.DEPLOYED, wrist.stateMachine().goal());
        assertEquals(ArmState.PREP, arm.stateMachine().goal());
    }

    @Test
    void mechanismFactoriesBuildInParallelDuringStartupRegistration() {
        String propertyKey = "athena.mechanism.factoryBuildParallelism";
        String previous = System.getProperty(propertyKey);
        System.setProperty(propertyKey, "4");
        try {
            int factoryCount = 4;
            AtomicInteger activeBuilders = new AtomicInteger();
            AtomicInteger maxConcurrentBuilders = new AtomicInteger();
            CountDownLatch started = new CountDownLatch(factoryCount);

            RobotCore<FakeDrivetrain> core = new RobotCore<>(
                    RobotCoreConfig
                            .create()
                            .drivetrain(__ -> new FakeDriveConfig())
                            .mechanisms(mechanisms -> {
                                for (int i = 0; i < factoryCount; i++) {
                                    mechanisms.existing(parallelFactory(
                                            "parallel-" + i,
                                            started,
                                            activeBuilders,
                                            maxConcurrentBuilders));
                                }
                            })
                            .build());

            assertTrue(
                    maxConcurrentBuilders.get() > 1,
                    "Expected startup mechanism factories to overlap in execution.");
            for (int i = 0; i < factoryCount; i++) {
                assertTrue(core.mechanism("parallel-" + i) != null);
            }
        } finally {
            if (previous == null) {
                System.clearProperty(propertyKey);
            } else {
                System.setProperty(propertyKey, previous);
            }
        }
    }

    @Test
    void startupCanDeferMechanismRegistrationUntilDisabledPeriodic() {
        String deferInitKey = "athena.startup.deferNonDriveInit";
        String deferMechKey = "athena.startup.deferMechanismRegistration";
        String budgetKey = "athena.startup.deferredInitBudgetMs";
        String previousDeferInit = System.getProperty(deferInitKey);
        String previousDeferMech = System.getProperty(deferMechKey);
        String previousBudget = System.getProperty(budgetKey);
        System.setProperty(deferInitKey, "true");
        System.setProperty(deferMechKey, "true");
        System.setProperty(budgetKey, "100");
        try {
            RobotCore<FakeDrivetrain> core = new RobotCore<>(
                    RobotCoreConfig
                            .create()
                            .drivetrain(__ -> new FakeDriveConfig())
                            .mechanisms(mechanisms -> mechanisms
                                    .mechanism(MechanismConfig.generic("deferred-startup-mechanism")))
                            .build());

            assertNull(core.mechanism("deferred-startup-mechanism"));
            core.robotInit();
            assertNull(core.mechanism("deferred-startup-mechanism"));
            core.disabledPeriodic();
            assertNotNull(core.mechanism("deferred-startup-mechanism"));
        } finally {
            restoreProperty(deferInitKey, previousDeferInit);
            restoreProperty(deferMechKey, previousDeferMech);
            restoreProperty(budgetKey, previousBudget);
        }
    }

    private static RegisterableMechanismFactory parallelFactory(
            String mechanismName,
            CountDownLatch started,
            AtomicInteger activeBuilders,
            AtomicInteger maxConcurrentBuilders) {
        return new RegisterableMechanismFactory() {
            @Override
            public RegisterableMechanism build() {
                int active = activeBuilders.incrementAndGet();
                maxConcurrentBuilders.accumulateAndGet(active, Math::max);
                started.countDown();
                try {
                    started.await(250, TimeUnit.MILLISECONDS);
                    Thread.sleep(50);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                } finally {
                    activeBuilders.decrementAndGet();
                }
                return MechanismConfig.generic(mechanismName).build();
            }

            @Override
            public List<Mechanism> flattenForRegistration() {
                return List.of();
            }
        };
    }

    private static void restoreProperty(String key, String value) {
        if (value == null) {
            System.clearProperty(key);
            return;
        }
        System.setProperty(key, value);
    }

    private static StatefulMechanism<DirectMechanismState> mechanism(DirectMechanismState defaultState) {
        return MechanismConfig.stateMachineGeneric("direct", defaultState)
                .control(config -> config.setpointAsOutput(true))
                .build();
    }

    private static StatefulMechanism<ArmState> mechanism(ArmState defaultState) {
        return MechanismConfig.stateMachineGeneric("arm", defaultState)
                .control(config -> config.setpointAsOutput(true))
                .build();
    }

    private static StatefulMechanism<WristState> mechanism(WristState defaultState) {
        return MechanismConfig.stateMachineGeneric("wrist", defaultState)
                .control(config -> config.setpointAsOutput(true))
                .build();
    }

    private static StatefulMechanism<SharedMechanismState> mechanism(
            String name,
            SharedMechanismState defaultState) {
        return MechanismConfig.stateMachineGeneric(name, defaultState)
                .control(config -> config.setpointAsOutput(true))
                .build();
    }

    private static final class FakeDriveConfig implements RobotDrivetrain.RobotDrivetrainConfig<FakeDrivetrain> {
        @Override
        public FakeDrivetrain build() {
            return new FakeDrivetrain();
        }
    }

    private static final class FakeDrivetrain extends SubsystemBase
            implements RobotDrivetrain<FakeDrivetrain>, RobotDrivetrainLocalizationFactory {

        private final RobotSpeeds robotSpeeds = new RobotSpeeds(5.0, 5.0);
        private final MotionLimits limits = new MotionLimits();
        private final FakeControlSection controlSection = new FakeControlSection();
        private final FakeSpeedsSection speedsSection = new FakeSpeedsSection(robotSpeeds);
        private final FakeHardwareSection hardwareSection = new FakeHardwareSection();
        private final FakeSysIdSection sysIdSection = new FakeSysIdSection();
        private final FakeImuSection imuSection = new FakeImuSection();
        private final FakeSimulationSection simulationSection = new FakeSimulationSection();

        @Override
        public FakeDrivetrain control(Consumer<ControlSection> section) {
            section.accept(controlSection);
            return this;
        }

        @Override
        public ControlSection control() {
            return controlSection;
        }

        @Override
        public FakeDrivetrain speeds(Consumer<SpeedsSection> section) {
            section.accept(speedsSection);
            return this;
        }

        @Override
        public SpeedsSection speeds() {
            return speedsSection;
        }

        @Override
        public FakeDrivetrain modules(Consumer<ModulesSection> section) {
            return this;
        }

        @Override
        public ModulesSection modules() {
            return new ModulesSection() {};
        }

        @Override
        public FakeDrivetrain hardware(Consumer<HardwareSection> section) {
            section.accept(hardwareSection);
            return this;
        }

        @Override
        public HardwareSection hardware() {
            return hardwareSection;
        }

        @Override
        public FakeDrivetrain sysId(Consumer<SysIdSection> section) {
            section.accept(sysIdSection);
            return this;
        }

        @Override
        public SysIdSection sysId() {
            return sysIdSection;
        }

        @Override
        public FakeDrivetrain imu(Consumer<ImuSection> section) {
            section.accept(imuSection);
            return this;
        }

        @Override
        public ImuSection imu() {
            return imuSection;
        }

        @Override
        public FakeDrivetrain simulation(Consumer<SimulationSection> section) {
            section.accept(simulationSection);
            return this;
        }

        @Override
        public SimulationSection simulation() {
            return simulationSection;
        }

        @Override
        public RobotSpeeds robotSpeeds() {
            return robotSpeeds;
        }

        @Override
        public void update() {
            // no-op
        }

        @Override
        public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
            return node;
        }

        @Override
        public RobotLocalization<?> createLocalization(RobotLocalizationConfig config) {
            return null;
        }

        private static final class FakeControlSection implements ControlSection {
            @Override
            public Command command(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
                return Commands.run(() -> {
                    xInput.getAsDouble();
                    yInput.getAsDouble();
                    thetaInput.getAsDouble();
                });
            }

            @Override
            public void defaultCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
            }

            @Override
            public void reset() {
            }

            @Override
            public void stop() {
            }
        }

        private static final class FakeSpeedsSection implements SpeedsSection {
            private final RobotSpeeds speeds;

            FakeSpeedsSection(RobotSpeeds speeds) {
                this.speeds = speeds;
            }

            @Override
            public MotionLimits limits() {
                return new MotionLimits();
            }

            @Override
            public ChassisSpeeds get(String source) {
                return speeds.getSpeeds(source);
            }

            @Override
            public ChassisSpeeds getInput(String source) {
                return speeds.getInputSpeeds(source);
            }

            @Override
            public SpeedsSection set(String source, ChassisSpeeds speeds) {
                this.speeds.setSpeeds(source, speeds);
                return this;
            }

            @Override
            public SpeedsSection set(String source, double x, double y, double theta) {
                this.speeds.setSpeeds(source, x, y, theta);
                return this;
            }

            @Override
            public SpeedsSection stop(String source) {
                speeds.stopSpeeds(source);
                return this;
            }

            @Override
            public SpeedsSection stop() {
                speeds.stop();
                return this;
            }

            @Override
            public SpeedsSection enabled(String source, boolean enabled) {
                speeds.setSpeedSourceState(source, enabled);
                return this;
            }

            @Override
            public boolean enabled(String source) {
                return speeds.isSpeedsSourceActive(source);
            }

            @Override
            public double maxVelocity() {
                return speeds.getMaxVelocity();
            }

            @Override
            public double maxAngularVelocity() {
                return speeds.getMaxAngularVelocity();
            }

            @Override
            public Set<String> sources() {
                return speeds.getSpeedSources();
            }
        }

        private static final class FakeHardwareSection implements HardwareSection {
            @Override
            public void neutralMode(MotorNeutralMode mode) {
            }
        }

        private static final class FakeSysIdSection implements SysIdSection {
            private double rampRateVoltsPerSecond;
            private double stepVoltage;
            private double timeoutSeconds;
            private double voltageLimit;

            @Override
            public double rampRateVoltsPerSecond() {
                return rampRateVoltsPerSecond;
            }

            @Override
            public void rampRateVoltsPerSecond(double voltsPerSecond) {
                rampRateVoltsPerSecond = voltsPerSecond;
            }

            @Override
            public double stepVoltage() {
                return stepVoltage;
            }

            @Override
            public void stepVoltage(double volts) {
                stepVoltage = volts;
            }

            @Override
            public double timeoutSeconds() {
                return timeoutSeconds;
            }

            @Override
            public void timeoutSeconds(double seconds) {
                timeoutSeconds = seconds;
            }

            @Override
            public double voltageLimit() {
                return voltageLimit;
            }

            @Override
            public void voltageLimit(double volts) {
                voltageLimit = volts;
            }

            @Override
            public boolean active() {
                return false;
            }

            @Override
            public Command quasistatic(SysIdRoutine.Direction direction) {
                return Commands.none();
            }

            @Override
            public Command dynamic(SysIdRoutine.Direction direction) {
                return Commands.none();
            }
        }

        private static final class FakeImuSection implements ImuSection {
            @Override
            public Imu device() {
                return null;
            }
        }

        private static final class FakeSimulationSection implements SimulationSection {
            private boolean enabled;
            private Pose2d pose = new Pose2d();

            @Override
            public boolean enabled() {
                return enabled;
            }

            @Override
            public Pose2d pose() {
                return pose;
            }

            @Override
            public void pose(Pose2d pose) {
                this.pose = pose != null ? pose : new Pose2d();
            }
        }
    }
}
