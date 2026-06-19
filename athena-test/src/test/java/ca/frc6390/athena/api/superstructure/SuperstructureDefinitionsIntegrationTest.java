package ca.frc6390.athena.api.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.statespec.StateId;
import ca.frc6390.athena.mechanisms.statespec.StateSet;

class SuperstructureDefinitionsIntegrationTest {
    private enum TurretState implements SetpointProvider<Double> {
        Off(90.0),
        Aim(50.0);

        private final double setpoint;

        TurretState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    private enum HoodState implements SetpointProvider<Double> {
        Stow(80.0),
        Aim(55.0);

        private final double setpoint;

        HoodState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    private record TurretTuple(TurretState turret, HoodState hood) {}

    private enum TurretSuperState implements SetpointProvider<TurretTuple> {
        Stowed(new TurretTuple(TurretState.Off, HoodState.Stow)),
        Aim(new TurretTuple(TurretState.Aim, HoodState.Aim));

        private final TurretTuple setpoint;

        TurretSuperState(TurretTuple setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public TurretTuple getSetpoint() {
            return setpoint;
        }
    }

    private record TopTuple(TurretSuperState turretSuper) {}

    private enum TopState implements SetpointProvider<TopTuple> {
        Home(new TopTuple(TurretSuperState.Stowed)),
        Aim(new TopTuple(TurretSuperState.Aim));

        private final TopTuple setpoint;

        TopState(TopTuple setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public TopTuple getSetpoint() {
            return setpoint;
        }
    }

    private enum SimpleState implements SetpointProvider<Integer> {
        Home(0),
        Aim(1);

        private final int setpoint;

        SimpleState(int setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Integer getSetpoint() {
            return setpoint;
        }
    }

    private static final class SuperStates extends StateSet {
        static final StateId Home = state("Home", 0.0);
        static final StateId Aim = state("Aim", 1.0);
    }

    private interface SimpleSuperConfig extends SuperstructureConfig<SimpleState, Integer> {
        @Override
        default String name() {
            return "simpleClass";
        }

        @Override
        default SimpleState initialState() {
            return SimpleState.Home;
        }
    }

    private interface PlainSuperSpec {
        static void configure(SuperstructureSpec<SimpleState, Integer> sup) {
            identity(sup);
            inputs(sup);
        }

        private static void identity(SuperstructureSpec<SimpleState, Integer> sup) {
            sup.name("plainSuperSpec");
            sup.initialState(SimpleState.Home);
        }

        private static void inputs(SuperstructureSpec<SimpleState, Integer> sup) {
            sup.inputs().boolVal("allowAim", true);
        }
    }

    @Test
    void nestedV2SuperstructureBuildsDirectRuntime() {
        MechanismDefinition turretDefinition = Mechanisms.stateful("turret", TurretState.Off).definition();
        MechanismDefinition hoodDefinition = Mechanisms.stateful("hood", HoodState.Stow).definition();

        var turretSuperDefinition = Superstructures.<TurretSuperState, TurretTuple>stateful(
            "turretSuper",
            TurretSuperState.Stowed)
            .mechanisms(mechanisms -> mechanisms
                .mechanism(turretDefinition, TurretTuple::turret)
                .mechanism(hoodDefinition, TurretTuple::hood))
            .definition();

        var topDefinition = Superstructures.<TopState, TopTuple>stateful("top", TopState.Home)
            .mechanisms(mechanisms -> mechanisms.superstructure(turretSuperDefinition, TopTuple::turretSuper))
            .definition();

        @SuppressWarnings("unchecked")
        SuperstructureMechanism<TopState, TopTuple> top =
            (SuperstructureMechanism<TopState, TopTuple>) SuperstructureDefinitions.build(topDefinition);

        SuperstructureMechanism<TurretSuperState, TurretTuple> turretSuper =
            top.mechanisms().superstructure(TopTuple::turretSuper);
        StatefulMechanism<TurretState> turret = turretSuper.mechanisms().generic(TurretTuple::turret);
        StatefulMechanism<HoodState> hood = turretSuper.mechanisms().generic(TurretTuple::hood);

        assertEquals(TopState.Home, top.stateMachine().goal());
        assertEquals(TurretSuperState.Stowed, turretSuper.stateMachine().goal());
        assertEquals(TurretState.Off, turret.stateMachine().goal());
        assertEquals(HoodState.Stow, hood.stateMachine().goal());

        top.stateMachine().queue(TopState.Aim);
        tick(top, turretSuper, turret, hood, 6);

        assertEquals(TopState.Aim, top.stateMachine().goal());
        assertEquals(TurretSuperState.Aim, turretSuper.stateMachine().goal());
        assertEquals(TurretState.Aim, turret.stateMachine().goal());
        assertEquals(HoodState.Aim, hood.stateMachine().goal());
    }

    @Test
    void directV2SuperstructureRunsInputsConstraintsAndHooks() {
        AtomicBoolean allowAim = new AtomicBoolean(false);
        AtomicInteger enterCount = new AtomicInteger();
        AtomicInteger periodicCount = new AtomicInteger();
        AtomicInteger exitCount = new AtomicInteger();
        AtomicInteger transitionCount = new AtomicInteger();
        AtomicInteger robotPeriodicCount = new AtomicInteger();
        AtomicInteger initCount = new AtomicInteger();
        AtomicInteger teleCount = new AtomicInteger();

        var definition = Superstructures.<SimpleState, Integer>stateful("simple", SimpleState.Home)
            .inputs(inputs -> inputs
                .boolVal("allowAim", allowAim::get)
                .doubleVal("targetAngle", 35.0))
            .behavior(behavior -> behavior
                .constraints(constraints -> constraints.state(SimpleState.Aim, ctx -> ctx.input("allowAim")))
                .hooks(hooks -> hooks
                    .onStateEnter(ctx -> enterCount.incrementAndGet(), SimpleState.Aim)
                    .onStatePeriodic(ctx -> {
                        if (ctx.input("allowAim") && ctx.doubleInput("targetAngle") == 35.0) {
                            periodicCount.incrementAndGet();
                        }
                    }, SimpleState.Aim)
                    .onStateExit(ctx -> exitCount.incrementAndGet(), SimpleState.Aim)
                    .onStateTransition((ctx, from, to) -> transitionCount.incrementAndGet(), SimpleState.Home, SimpleState.Aim)
                    .onRobotPeriodic(ctx -> robotPeriodicCount.incrementAndGet())
                    .onInit(ctx -> initCount.incrementAndGet())
                    .onTeleopPeriodic(ctx -> teleCount.incrementAndGet(), SimpleState.Aim)))
            .definition();

        @SuppressWarnings("unchecked")
        SuperstructureMechanism<SimpleState, Integer> superstructure =
            (SuperstructureMechanism<SimpleState, Integer>) SuperstructureDefinitions.build(definition);

        superstructure.runInitHooks();
        assertEquals(1, initCount.get());

        superstructure.stateMachine().queue(SimpleState.Aim);
        tick(superstructure, 3);
        assertEquals(SimpleState.Home, superstructure.stateMachine().goal());
        assertEquals(0, enterCount.get());

        allowAim.set(true);
        tick(superstructure, 4);
        assertEquals(SimpleState.Aim, superstructure.stateMachine().goal());
        assertEquals(1, enterCount.get());
        assertEquals(1, transitionCount.get());
        assertTrue(periodicCount.get() > 0);
        assertTrue(robotPeriodicCount.get() > 0);

        superstructure.runLifecycleHooks(RobotCoreHooks.Phase.TELEOP_PERIODIC);
        assertEquals(1, teleCount.get());

        superstructure.stateMachine().queue(SimpleState.Home);
        tick(superstructure, 3);
        assertEquals(SimpleState.Home, superstructure.stateMachine().goal());
        assertEquals(1, exitCount.get());
    }

    @Test
    void stateSetSuperstructureBuildsAndRunsWithoutEnumState() {
        AtomicBoolean allowAim = new AtomicBoolean(false);
        AtomicInteger enterCount = new AtomicInteger();

        var definition = Superstructures.<StateId, Double>stateful("stateSet", SuperStates.Home)
            .inputs(inputs -> inputs.boolVal("allowAim", allowAim::get))
            .behavior(behavior -> behavior
                .constraints(constraints -> constraints.state(SuperStates.Aim, ctx -> ctx.input("allowAim")))
                .hooks(hooks -> hooks.onStateEnter(ctx -> enterCount.incrementAndGet(), SuperStates.Aim)))
            .definition();

        @SuppressWarnings("unchecked")
        SuperstructureMechanism<StateId, Double> superstructure =
            (SuperstructureMechanism<StateId, Double>) SuperstructureDefinitions.build(definition);

        assertEquals(SuperStates.Home, superstructure.stateMachine().goal());
        assertEquals(0.0, superstructure.stateMachine().setpoint(), 1e-9);

        superstructure.stateMachine().queue(SuperStates.Aim);
        tickStateSet(superstructure, 3);
        assertEquals(SuperStates.Home, superstructure.stateMachine().goal());
        assertEquals(0, enterCount.get());

        allowAim.set(true);
        tickStateSet(superstructure, 4);
        assertEquals(SuperStates.Aim, superstructure.stateMachine().goal());
        assertEquals(1.0, superstructure.stateMachine().setpoint(), 1e-9);
        assertEquals(1, enterCount.get());
    }

    @Test
    void structuredSuperstructureCanBeBuiltFromDeclarationClass() {
        var definition = SuperstructureDefinitions.structured(SimpleSuperConfig.class);

        assertEquals("simpleClass", definition.name());
        assertEquals(SimpleState.Home, definition.initialState().orElseThrow());
        assertEquals("Home", definition.initialStateName());
    }

    @Test
    void staticConfigureSpecBuildsSuperstructureFromPlainDeclarationClass() {
        var definition = SuperstructureDefinitions.structured(PlainSuperSpec.class);

        assertEquals("plainSuperSpec", definition.name());
        assertEquals(SimpleState.Home, definition.initialState().orElseThrow());
        assertEquals("Home", definition.initialStateName());
        assertEquals(1, definition.inputs().size());
    }

    private static void tick(
            SuperstructureMechanism<TopState, TopTuple> top,
            SuperstructureMechanism<TurretSuperState, TurretTuple> turretSuper,
            StatefulMechanism<TurretState> turret,
            StatefulMechanism<HoodState> hood,
            int cycles) {
        for (int i = 0; i < cycles; i++) {
            top.periodic();
            turretSuper.periodic();
            turret.periodic();
            hood.periodic();
        }
    }

    private static void tick(
            SuperstructureMechanism<SimpleState, Integer> superstructure,
            int cycles) {
        for (int i = 0; i < cycles; i++) {
            superstructure.periodic();
        }
    }

    private static void tickStateSet(
            SuperstructureMechanism<StateId, Double> superstructure,
            int cycles) {
        for (int i = 0; i < cycles; i++) {
            superstructure.periodic();
        }
    }
}
