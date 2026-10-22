package ca.frc6390.athena.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.MotorGroupEvent;
import ca.frc6390.athena.hardware.runtime.MotorGroupPolicy;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MotorFailover;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class MotorFailoverIntegrationTest {
    private final List<RobotRuntime> runtimes = new ArrayList<>();

    @AfterEach
    void closeRuntimes() {
        runtimes.forEach(RobotRuntime::close);
    }

    @Test
    void deadLeaderPromotesTheLowestIdFollowerAndRepointsTheOthers() {
        Harness harness = harness(failover -> failover.promoteLeader().dropFailedFollowers());
        harness.cycle();
        assertEquals(3.0, harness.motor(harness.shooter.leader).commandValue(), 1.0e-9);
        assertEquals(3.0, harness.motor(harness.shooter.follower).commandValue(), 1.0e-9);
        assertEquals(-3.0, harness.motor(harness.shooter.invertedFollower).commandValue(), 1.0e-9);

        harness.motor(harness.shooter.leader).connected(false);
        harness.cycle();

        SimMotorHandle promoted = harness.motor(harness.shooter.follower);
        SimMotorHandle invertedFollower = harness.motor(harness.shooter.invertedFollower);
        assertNull(promoted.followedLeader());
        assertEquals(SimMotorHandle.CommandKind.VOLTAGE, promoted.commandKind());
        assertEquals(3.0, promoted.commandValue(), 1.0e-9);
        assertSame(promoted, invertedFollower.followedLeader());
        assertEquals(-3.0, invertedFollower.commandValue(), 1.0e-9);
        assertEquals(harness.shooter.follower, harness.shooter.failover.leader());
        assertEquals(2, harness.shooter.failover.group().orElseThrow().aliveCount());
        assertEquals(List.of(MotorGroupEvent.Kind.LEADER_PROMOTED), harness.kinds());
        assertEquals(harness.shooter.leader, harness.events.get(0).motor());
        assertEquals(harness.shooter.follower, harness.events.get(0).leader());

        harness.cycle();
        assertEquals(3.0, promoted.commandValue(), 1.0e-9);
        assertEquals(-3.0, invertedFollower.commandValue(), 1.0e-9);
    }

    @Test
    void promotingAnInvertedFollowerKeepsEveryShaftTurningTheSameWay() {
        Harness harness = harness((shooter, failover) -> failover.promoteLeader(shooter.invertedFollower));
        harness.cycle();

        harness.motor(harness.shooter.leader).connected(false);
        harness.cycle();

        SimMotorHandle promoted = harness.motor(harness.shooter.invertedFollower);
        SimMotorHandle follower = harness.motor(harness.shooter.follower);
        assertNull(promoted.followedLeader());
        assertEquals(-3.0, promoted.commandValue(), 1.0e-9);
        assertSame(promoted, follower.followedLeader());
        assertEquals(3.0, follower.commandValue(), 1.0e-9);
    }

    @Test
    void promotedLeaderKeepsTheDeclaredLeadersIntegratedPositionContinuous() {
        Harness harness = harness(failover -> failover.promoteLeader().dropFailedFollowers());
        harness.motor(harness.shooter.leader).state(10.0, 0.0);
        harness.motor(harness.shooter.follower).state(3.0, 0.0);
        harness.cycle();
        assertEquals(10.0, harness.shooter.leader.position().in(Rotations), 1.0e-9);

        harness.motor(harness.shooter.leader).connected(false);
        harness.cycle();
        assertEquals(10.0, harness.shooter.leader.position().in(Rotations), 1.0e-9);

        harness.motor(harness.shooter.follower).state(4.5, 0.0);
        assertEquals(11.5, harness.shooter.leader.position().in(Rotations), 1.0e-9);
    }

    @Test
    void softwareFollowerOnAnotherBusIsPromotedAndNoLongerMirrored() {
        SimulationSession simulation = SimulationSession.create();
        CrossBusShooter shooter = new CrossBusShooter();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(shooter);
        runtimes.add(runtime);
        runtime.request(shooter.spin);
        runtime.teleopPeriodic(0.0, 0.02);
        assertEquals(-6.0, simulation.motor(shooter.follower).commandValue(), 1.0e-9);

        simulation.motor(shooter.leader).connected(false);
        runtime.teleopPeriodic(0.02, 0.02);
        runtime.teleopPeriodic(0.04, 0.02);

        assertEquals(-6.0, simulation.motor(shooter.follower).commandValue(), 1.0e-9);
        assertEquals(shooter.follower, shooter.failover.leader());
        assertTrue(runtime.hardwareGraph().softwareFollowers(shooter.leader).isEmpty());
    }

    @Test
    void deadFollowerIsDroppedWithDeratedOutputAndRejoinsWhenItRecovers() throws InterruptedException {
        Harness harness = harness(failover -> failover.dropFailedFollowers().derateOutput(0.5).recoverAfter(2));
        harness.cycle();

        harness.motor(harness.shooter.invertedFollower).connected(false);
        harness.cycle();

        assertEquals(1.5, harness.motor(harness.shooter.leader).commandValue(), 1.0e-9);
        assertEquals(1.5, harness.motor(harness.shooter.follower).commandValue(), 1.0e-9);
        assertEquals(0.0, harness.motor(harness.shooter.invertedFollower).commandValue(), 1.0e-9);
        assertTrue(harness.shooter.failover.group().orElseThrow().degraded());
        assertFalse(harness.shooter.failover.alive(harness.shooter.invertedFollower));
        assertEquals(List.of(MotorGroupEvent.Kind.FOLLOWER_DROPPED), harness.kinds());

        harness.motor(harness.shooter.invertedFollower).connected(true);
        Thread.sleep(110L); // the graph retries a failed refresh after a 100 ms backoff
        harness.cycle();
        harness.cycle();
        harness.cycle();

        assertEquals(3.0, harness.motor(harness.shooter.leader).commandValue(), 1.0e-9);
        assertEquals(-3.0, harness.motor(harness.shooter.invertedFollower).commandValue(), 1.0e-9);
        assertSame(harness.motor(harness.shooter.leader), harness.motor(harness.shooter.invertedFollower).followedLeader());
        assertFalse(harness.shooter.failover.group().orElseThrow().degraded());
        assertEquals(List.of(MotorGroupEvent.Kind.FOLLOWER_DROPPED, MotorGroupEvent.Kind.MOTOR_RECOVERED),
                harness.kinds());
    }

    @Test
    void recoveredLeaderRejoinsAsAFollowerOfThePromotedMotor() throws InterruptedException {
        Harness harness = harness(failover -> failover.promoteLeader().recoverAfter(1));
        harness.cycle();
        harness.motor(harness.shooter.leader).connected(false);
        harness.cycle();

        harness.motor(harness.shooter.leader).connected(true);
        Thread.sleep(110L); // the graph retries a failed refresh after a 100 ms backoff
        harness.cycle();
        harness.cycle();

        SimMotorHandle promoted = harness.motor(harness.shooter.follower);
        assertEquals(harness.shooter.follower, harness.shooter.failover.leader());
        assertSame(promoted, harness.motor(harness.shooter.leader).followedLeader());
        assertEquals(3.0, harness.motor(harness.shooter.leader).commandValue(), 1.0e-9);
        assertEquals(3.0, promoted.commandValue(), 1.0e-9);
    }

    @Test
    void strictPolicyDisablesTheMechanismWhenAFollowerDies() {
        Harness harness = harness(MotorFailover::disableMechanism);
        harness.cycle();

        harness.motor(harness.shooter.invertedFollower).connected(false);
        harness.cycle();

        assertEquals(0.0, harness.motor(harness.shooter.leader).commandValue(), 1.0e-9);
        assertEquals(0.0, harness.motor(harness.shooter.follower).commandValue(), 1.0e-9);
        assertTrue(harness.shooter.failover.group().orElseThrow().failed());
        assertEquals(List.of(MotorGroupEvent.Kind.GROUP_FAILED), harness.kinds());
    }

    @Test
    void leaderDeathWithoutPromotionDisablesTheMechanism() {
        Harness harness = harness(failover -> failover.dropFailedFollowers());
        harness.cycle();

        harness.motor(harness.shooter.leader).connected(false);
        harness.cycle();

        assertEquals(SimMotorHandle.CommandKind.NEUTRAL, harness.motor(harness.shooter.follower).commandKind());
        assertEquals(harness.shooter.leader, harness.shooter.failover.leader());
        assertEquals(List.of(MotorGroupEvent.Kind.GROUP_FAILED), harness.kinds());
    }

    @Test
    void minimumAliveFailsTheGroupEvenWhenItCouldPromote() {
        Harness harness = harness(failover -> failover.promoteLeader().dropFailedFollowers().minimumAlive(3));
        harness.cycle();

        harness.motor(harness.shooter.invertedFollower).connected(false);
        harness.cycle();

        assertEquals(0.0, harness.motor(harness.shooter.leader).commandValue(), 1.0e-9);
        assertEquals(List.of(MotorGroupEvent.Kind.GROUP_FAILED), harness.kinds());
    }

    @Test
    void faultedControllerCountsAsDead() {
        Harness harness = harness(failover -> failover.promoteLeader().dropFailedFollowers());
        harness.cycle();

        harness.motor(harness.shooter.leader).faulted(true);
        harness.cycle();

        assertEquals(harness.shooter.follower, harness.shooter.failover.leader());
        assertEquals(3.0, harness.motor(harness.shooter.follower).commandValue(), 1.0e-9);
    }

    @Test
    void groupDeclaredAfterTheMotorsResolvedTakesOverTheLeaderAndItsEncoder() {
        SimulationSession simulation = SimulationSession.create();
        PlainShooter shooter = new PlainShooter();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(shooter);
        runtimes.add(runtime);
        runtime.request(shooter.spin);
        simulation.motor(shooter.leader).state(7.0, 0.0);
        runtime.teleopPeriodic(0.0, 0.02);
        assertEquals(7.0, shooter.encoder.positionAngle().in(Rotations), 1.0e-9);

        runtime.hardwareGraph().motorGroup(new MotorGroupPolicy(
                shooter.leader, List.of(), true, true, 1, 1.0, 1.0, true, 5));
        simulation.motor(shooter.leader).connected(false);
        runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(6.0, simulation.motor(shooter.follower).commandValue(), 1.0e-9);
        assertNull(simulation.motor(shooter.follower).followedLeader());
        assertEquals(7.0, shooter.encoder.positionAngle().in(Rotations), 1.0e-9);
        assertEquals(7.0, shooter.leader.position().in(Rotations), 1.0e-9);
    }

    private Harness harness(UnaryOperator<MotorFailover> policy) {
        return harness((shooter, failover) -> policy.apply(failover));
    }

    private Harness harness(BiFunction<Shooter, MotorFailover, MotorFailover> policy) {
        Harness harness = new Harness(policy);
        runtimes.add(harness.runtime);
        return harness;
    }

    private static final class Harness {
        private final SimulationSession simulation = SimulationSession.create();
        private final Shooter shooter;
        private final RobotRuntime runtime;
        private final List<MotorGroupEvent> events = new ArrayList<>();
        private double now;

        private Harness(BiFunction<Shooter, MotorFailover, MotorFailover> policy) {
            shooter = new Shooter((owner, failover) -> policy.apply(owner, failover
                    .onFailure(events::add)
                    .onRecovery(events::add)));
            runtime = RobotRuntime.simulated(simulation).register(shooter);
            runtime.request(shooter.spin);
        }

        private void cycle() {
            runtime.teleopPeriodic(now, 0.02);
            now += 0.02;
        }

        private SimMotorHandle motor(MotorDevice device) {
            return simulation.motor(device);
        }

        private List<MotorGroupEvent.Kind> kinds() {
            return events.stream().map(MotorGroupEvent::kind).toList();
        }
    }

    private static final class Shooter implements Mechanism {
        private final MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 31);
        private final MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 32).follow(leader);
        private final MotorDevice invertedFollower =
                MotorDevice.of(MotorKinds.KRAKEN_X60, 33).follow(leader).inverted();
        private final MotorFailover failover;
        private final Action spin = leader.voltage(Volts.of(3.0));

        private Shooter(BiFunction<Shooter, MotorFailover, MotorFailover> policy) {
            failover = policy.apply(this, MotorFailover.of(leader));
        }
    }

    private static final class PlainShooter implements Mechanism {
        private final MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 51);
        private final MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 52).follow(leader);
        private final EncoderDevice encoder = leader.encoder();
        private final Action spin = leader.voltage(Volts.of(6.0));
    }

    private static final class CrossBusShooter implements Mechanism {
        private final MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 41).canbus("canivore");
        private final MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 42).follow(leader).inverted();
        private final MotorFailover failover = MotorFailover.of(leader).promoteLeader();
        private final Action spin = leader.voltage(Volts.of(6.0));
    }
}
