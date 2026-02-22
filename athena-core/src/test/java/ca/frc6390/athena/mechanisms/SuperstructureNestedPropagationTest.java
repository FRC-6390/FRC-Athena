package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

final class SuperstructureNestedPropagationTest {

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

    @Test
    void queuedTopStatePropagatesIntoNestedSuperAndKeepsNonZeroLeafSetpoints() {
        MechanismConfig<StatefulMechanism<TurretState>> turretConfig =
                MechanismConfig.stateMachineGeneric(TurretState.Off)
                        .control(c -> c.setpointAsOutput(true));
        MechanismConfig<StatefulMechanism<HoodState>> hoodConfig =
                MechanismConfig.stateMachineGeneric(HoodState.Stow)
                        .control(c -> c.setpointAsOutput(true));

        SuperstructureConfig<TurretSuperState, TurretTuple> turretSuperConfig =
                SuperstructureConfig.create(TurretSuperState.Stowed)
                        .mechanisms(m -> m
                                .mechanism(turretConfig, TurretTuple::turret)
                                .mechanism(hoodConfig, TurretTuple::hood))
                        .stateMachineDelay(0.0);

        SuperstructureConfig<TopState, TopTuple> topConfig =
                SuperstructureConfig.create(TopState.Home)
                        .mechanisms(m -> m.superstructure(turretSuperConfig, TopTuple::turretSuper))
                        .constraints(c -> c.state(
                                TopState.Aim,
                                ctx -> ctx.mechanisms()
                                        .superstructure(TopTuple::turretSuper)
                                        .stateMachine()
                                        .goal() == TurretSuperState.Aim))
                        .stateMachineDelay(0.0);

        SuperstructureMechanism<TopState, TopTuple> top = topConfig.build();
        SuperstructureMechanism<TurretSuperState, TurretTuple> turretSuper =
                top.mechanisms().superstructure(TopTuple::turretSuper);
        StatefulMechanism<TurretState> turret = turretSuper.mechanisms().generic(TurretTuple::turret);
        StatefulMechanism<HoodState> hood = turretSuper.mechanisms().generic(TurretTuple::hood);

        tick(top, turretSuper, turret, hood, 3);
        assertEquals(TopState.Home, top.stateMachine().goal());
        assertEquals(TurretSuperState.Stowed, turretSuper.stateMachine().goal());
        assertEquals(TurretState.Off, turret.stateMachine().goal());
        assertEquals(HoodState.Stow, hood.stateMachine().goal());

        top.stateMachine().queue(TopState.Aim);

        tick(top, turretSuper, turret, hood, 8);

        assertEquals(TurretSuperState.Aim, turretSuper.stateMachine().goal());
        assertEquals(TopState.Aim, top.stateMachine().goal());
        assertEquals(TurretState.Aim, turret.stateMachine().goal());
        assertEquals(HoodState.Aim, hood.stateMachine().goal());
        assertEquals(50.0, turret.setpoint(), 1e-9);
        assertEquals(55.0, hood.setpoint(), 1e-9);
        assertNotEquals(0.0, turret.setpoint(), 1e-9);
        assertNotEquals(0.0, hood.setpoint(), 1e-9);
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
}
