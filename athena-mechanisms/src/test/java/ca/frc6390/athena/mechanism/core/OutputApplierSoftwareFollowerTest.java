package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertNotSame;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.ControlRoute;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.runtime.MotorGroupPolicy;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.mechanism.effort.EffortCarrier;
import ca.frc6390.athena.mechanism.effort.EffortRequest;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class OutputApplierSoftwareFollowerTest {
    @Test
    void oneControlBindingCommandsEverySuppliedMotor() {
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        MotorDevice third = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);
        RecordingMotor firstHandle = new RecordingMotor(first);
        RecordingMotor secondHandle = new RecordingMotor(second);
        RecordingMotor thirdHandle = new RecordingMotor(third);
        ControlBinding<?> control = Controls.velocity(first, Units.Rotations).followers(second, third);
        ActionContext context = new ActionContext() {
            @Override
            public MotorHandle motor(MotorDevice device) {
                if (device.equals(first)) return firstHandle;
                if (device.equals(second)) return secondHandle;
                return thirdHandle;
            }
        };
        Output velocity = Outputs.velocity(42.0);

        OutputApplier.using(context).apply(new ResolvedOutput(OutputRequest.of(control, velocity), velocity));

        assertEquals(42.0, firstHandle.velocity, 1.0e-9);
        assertEquals(42.0, secondHandle.velocity, 1.0e-9);
        assertEquals(42.0, thirdHandle.velocity, 1.0e-9);
    }

    @Test
    void resolvedLeaderOutputReachesSoftwareFollowerInTheSameApply() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).canbus("rio");
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).canbus("canivore").follow(leader);
        RecordingMotor leaderHandle = new RecordingMotor(leader);
        RecordingMotor followerHandle = new RecordingMotor(follower);
        ActionContext context = new ActionContext() {
            @Override
            public MotorHandle motor(MotorDevice device) {
                return device.equals(leader) ? leaderHandle : followerHandle;
            }

            @Override
            public List<SoftwareMotorFollower> softwareFollowers(MotorDevice device) {
                return device.equals(leader)
                        ? List.of(new SoftwareMotorFollower(follower, followerHandle))
                        : List.of();
            }
        };
        Output voltage = Outputs.voltage(6.0);

        OutputApplier.using(context).apply(new ResolvedOutput(OutputRequest.of(leader, voltage), voltage));

        assertEquals(6.0, leaderHandle.voltage, 1.0e-9);
        assertEquals(6.0, followerHandle.voltage, 1.0e-9);
    }

    @Test
    void opposingCrossBusShooterKeepsTorqueAndCurrentCapsInBothDirections() {
        for (boolean closedLoop : new boolean[] {false, true}) {
            for (double direction : new double[] {1.0, -1.0}) {
                try (FollowerRig rig = new FollowerRig(false, false)) {
                    rig.leaderMotor.speed = direction * 50.0;
                    rig.followerMotor.speed = -direction * 50.0;
                    rig.apply(direction * 12.0, 40.0, closedLoop);

                    assertTrue(direction * rig.leaderMotor.voltage > 0.0);
                    assertEquals(-rig.leaderMotor.voltage, rig.followerMotor.voltage, 1e-9);
                    double leaderAmps = rig.current(rig.leaderMotor);
                    double followerAmps = rig.current(rig.followerMotor);
                    assertTrue(direction * leaderAmps > 0.0, "leader accelerates in commanded direction");
                    assertTrue(-direction * followerAmps > 0.0, "opposing follower accelerates its shaft");
                    assertEquals(40.0, Math.abs(leaderAmps), 1e-8);
                    assertEquals(40.0, Math.abs(followerAmps), 1e-8);
                    assertEquals(-direction * 50.0, rig.graph.motor(rig.follower).integratedVelocityRotationsPerSecond());
                    assertEquals(rig.followerMotor.voltage, rig.graph.motor(rig.follower).appliedVoltage());

                    rig.apply(-direction * 12.0, 40.0, closedLoop);
                    assertEquals(-40.0 * direction, rig.current(rig.leaderMotor), 1e-8,
                            "reverse demand brakes within the current cap before the shaft reverses");
                    assertEquals(40.0 * direction, rig.current(rig.followerMotor), 1e-8);
                }
            }
        }
    }

    @Test
    void hardwareFollowersStayVendorManagedAndControllerInversionIsNotAppliedTwice() {
        try (FollowerRig rig = new FollowerRig(true, true)) {
            // The controller exposes its already-phased integrated feedback, even when configured inverted.
            rig.leaderMotor.speed = 50.0;
            rig.followerMotor.speed = -50.0;
            rig.apply(12.0, 40.0, false);
            assertTrue(rig.graph.softwareFollowers(rig.leader).isEmpty());
            assertTrue(rig.leader.isInverted());
            assertTrue(rig.leaderMotor.voltage > 0.0);
            assertEquals(40.0, rig.current(rig.leaderMotor), 1e-8);
            assertEquals(0, rig.followerMotor.directVoltageCommands, "hardware follower is not commanded twice");
            assertEquals(-rig.leaderMotor.voltage, rig.followerMotor.appliedVoltage(), 1e-9);
        }
    }

    @Test
    void opposedNativeControllerPhasedFeedbackUsesOneAndStillBoundsTheFollower() {
        try (FollowerRig rig = new FollowerRig(true, true)) {
            rig.followerMotor.phasedNative = true;
            rig.leaderMotor.speed = 50.0;
            rig.followerMotor.speed = 50.0;
            var views = rig.graph.followerConstraints(rig.leader);
            assertSame(views, rig.graph.followerConstraints(rig.leader), "steady-state view is cached");
            assertEquals(1.0, views.get(0).handle().voltageCommandScale());
            rig.apply(12.0, 40.0, false);
            assertEquals(40.0, current(rig.follower, rig.followerMotor.appliedVoltage(), 50.0), 1e-8);
            assertEquals(0, rig.followerMotor.directVoltageCommands);
            assertTrue(rig.follower.isInverted());
        }
    }

    @Test
    void nestedSoftwareAndMixedNativeFollowersComposeDirectionAndAreCommandedOnce() {
        for (boolean nativeParent : new boolean[] {false, true}) {
            try (FollowerRig rig = new FollowerRig(nativeParent, false)) {
                MotorDevice child = MotorDevice.of(MotorKinds.FALCON_500, 35).canbus("third")
                        .follow(rig.follower).inverted();
                rig.graph.motor(child);
                RecordingMotor childMotor = rig.backend.motors.get(child.id());
                rig.leaderMotor.speed = 50.0;
                rig.followerMotor.speed = -50.0;
                childMotor.speed = 50.0;
                var views = rig.graph.followerConstraints(rig.leader);
                assertEquals(2, views.size());
                assertEquals(1.0, views.get(1).handle().voltageCommandScale(), 1e-9);
                rig.apply(12.0, 40.0, false);
                assertEquals(rig.leaderMotor.voltage, childMotor.voltage, 1e-9);
                assertEquals(40.0, current(child, childMotor.voltage, 50.0), 1e-8);
                assertEquals(1, childMotor.directVoltageCommands, "flattened software descendant dispatched once");
                assertEquals(nativeParent ? 0 : 1, rig.followerMotor.directVoltageCommands);
            }
        }
    }

    @Test
    void defaultContextNestedViewsRetainOnlyPhysicalStopTargets() {
        try (FollowerRig rig = new FollowerRig(false, false)) {
            MotorDevice child = MotorDevice.of(MotorKinds.FALCON_500, 37).canbus("third")
                    .follow(rig.follower).inverted();
            rig.graph.motor(child);
            RecordingMotor childMotor = rig.backend.motors.get(37);
            ActionContext custom = new ActionContext() {
                @Override public MotorHandle motor(MotorDevice device) { return rig.graph.motor(device); }
                @Override public List<SoftwareMotorFollower> softwareFollowers(MotorDevice device) {
                    return rig.graph.softwareFollowers(device);
                }
            };
            OutputApplier applier = OutputApplier.using(custom);
            Output voltage = Outputs.voltage(3.0);
            for (int cycle = 0; cycle < 200; cycle++) {
                applier.apply(new ResolvedOutput(OutputRequest.of(rig.leader, voltage), voltage));
            }
            assertEquals(200, childMotor.directVoltageCommands);
            assertEquals(3.0, childMotor.voltage);
            applier.stopAll();
            assertEquals(1, childMotor.stops, "generated descendant views never accumulate as stop targets");
            assertEquals(1, rig.followerMotor.stops);
            assertEquals(1, rig.leaderMotor.stops);
        }
    }

    @Test
    void activePowerCapsMoveDeviceAndHybridLoopsToHostVoltageForLeaderOrAnyFollower() {
        for (int constrained : new int[] {0, 1, 2}) {
            for (boolean hybrid : new boolean[] {false, true}) {
                FollowerBackend backend = new FollowerBackend();
                MotorDevice leader = MotorDevice.of(MotorKinds.NEO, 61).canbus("rio");
                if (constrained == 0) leader = leader.supplyCurrentLimit(Units.Amps.of(5));
                MotorDevice follower = MotorDevice.of(MotorKinds.NEO, 62)
                        .canbus(constrained == 1 ? "other" : "rio").follow(leader).inverted();
                if (constrained != 0) follower = follower.supplyCurrentLimit(Units.Amps.of(5));
                try (HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend))) {
                    graph.motor(follower);
                    RecordingMotor first = backend.motors.get(61);
                    RecordingMotor second = backend.motors.get(62);
                    first.speed = 50.0;
                    second.speed = -50.0;
                    ControlBinding<?> binding = Controls.velocity(leader, Units.Rotations)
                            .feedback(leader.encoder()).pid(0.4, 0.0, 0.0);
                    int[] feedforwardCalls = {0};
                    if (hybrid) binding = binding.loop(ControlLoops.arbitraryFeedforward(
                            ignored -> context -> { feedforwardCalls[0]++; return ControlOutput.voltage(1.5); }));
                    OutputApplier applier = OutputApplier.using(graph);
                    Output target = Outputs.velocity(80.0);
                    ResolvedOutput output = new ResolvedOutput(OutputRequest.of(binding, target), target);
                    applier.apply(output);
                    assertEquals(hybrid ? ControlRoute.HYBRID_CLOSED_LOOP : ControlRoute.DEVICE_CLOSED_LOOP,
                            first.lastRequest.route());
                    assertEquals(0, first.directVoltageCommands);

                    PowerRuntime power = new PowerRuntime();
                    power.policy(PowerPolicy.defaults().estimateBatteryResistance(false));
                    power.busVoltage(() -> 12.0);
                    power.cycle(List.of(leader, follower), graph, new RuntimeOverrides(), 0.0);
                    MotorDevice limited = constrained == 0 ? leader : follower;
                    assertTrue(Double.isFinite(power.capAmps(limited)));
                    applier.electrical(power);
                    applier.apply(output);
                    assertEquals(1, first.directVoltageCommands, "finite cap requires host voltage output");
                    assertEquals(constrained == 1 ? 1 : 0, second.directVoltageCommands,
                            "native followers receive no duplicate voltage command");
                    double limitedVoltage = constrained == 0 ? first.appliedVoltage() : second.appliedVoltage();
                    double limitedSpeed = constrained == 0 ? first.speed : second.speed;
                    assertEquals(power.capAmps(limited), Math.abs(current(limited, limitedVoltage, limitedSpeed)), 1e-8);
                    assertEquals(hybrid ? 2 : 0, feedforwardCalls[0], "hybrid feedforward evaluated once per apply");
                    int deviceCommands = first.velocityCommands;
                    applier.electrical(null);
                    applier.apply(output);
                    assertEquals(deviceCommands + 1, first.velocityCommands, "uncapped route offloads again");
                }
            }
        }
    }

    @Test
    void actionCurrentLimitAlsoMovesOtherwiseOffloadablePidToHostVoltage() {
        for (boolean nativeFollower : new boolean[] {false, true}) {
            try (FollowerRig rig = new FollowerRig(nativeFollower, false)) {
                rig.leaderMotor.speed = 50.0;
                rig.followerMotor.speed = -50.0;
                var binding = Controls.velocity(rig.leader, Units.Rotations).feedback(rig.leader.encoder()).pid(0.4, 0, 0);
                Output target = new LimitedVelocity(80.0, new EffortRequest(Units.Amps.of(40)));
                rig.applier.apply(new ResolvedOutput(OutputRequest.of(binding, target), target));
                assertEquals(0, rig.leaderMotor.velocityCommands, "per-Action effort must reach the host clamp");
                assertEquals(40.0, rig.current(rig.leaderMotor), 1e-8);
                assertEquals(-40.0, current(rig.follower, rig.followerMotor.appliedVoltage(), -50.0), 1e-8);
                assertEquals(nativeFollower ? 0 : 1, rig.followerMotor.directVoltageCommands);
            }
        }
    }

    @Test
    void nativeFollowerProfileCapAppliesThroughHostRouteAndCachedViewChangesOnBindingAndFailover() {
        try (FollowerRig rig = new FollowerRig(true, false)) {
            var original = rig.graph.followerConstraints(rig.leader);
            MotorDevice third = MotorDevice.of(MotorKinds.FALCON_500, 36).canbus("rio").follow(rig.leader)
                    .power(PowerProfile.of().sustained(Units.Amps.of(20)));
            rig.graph.motor(third);
            assertNotSame(original, rig.graph.followerConstraints(rig.leader));
            RecordingMotor thirdMotor = rig.backend.motors.get(36);
            thirdMotor.speed = 50.0;
            rig.leaderMotor.speed = 50.0;
            rig.followerMotor.speed = -50.0;
            PowerRuntime power = new PowerRuntime();
            power.busVoltage(() -> 12.0);
            power.cycle(List.of(rig.leader, rig.follower, third), rig.graph, new RuntimeOverrides(), 0.0);
            rig.applier.electrical(power);
            var binding = Controls.velocity(rig.leader, Units.Rotations).feedback(rig.leader.encoder()).pid(0.4, 0, 0);
            Output target = Outputs.velocity(80.0);
            rig.applier.apply(new ResolvedOutput(OutputRequest.of(binding, target), target));
            assertEquals(20.0, current(third, thirdMotor.appliedVoltage(), 50.0), 1e-8);
            assertEquals(0, thirdMotor.directVoltageCommands);
            rig.graph.motorGroup(new MotorGroupPolicy(rig.leader, List.of(third), true, true, 1, 0.5, 1, true, 1));
            var healthy = rig.graph.followerConstraints(rig.leader);
            rig.leaderMotor.failed = true;
            rig.graph.refreshInputs();
            var degraded = rig.graph.followerConstraints(rig.leader);
            assertNotSame(healthy, degraded);
            assertEquals(0.5, degraded.stream().filter(f -> f.device().equals(third)).findFirst().orElseThrow()
                    .handle().voltageCommandScale());
            assertTrue(degraded.stream().noneMatch(f -> f.device().equals(rig.leader)));
            rig.leaderMotor.failed = false;
            rig.graph.refreshInputs();
            var recovered = rig.graph.followerConstraints(rig.leader);
            assertNotSame(degraded, recovered);
            assertTrue(recovered.stream().anyMatch(f -> f.device().equals(rig.leader)));
            assertSame(recovered, rig.graph.followerConstraints(rig.leader));
        }
    }

    private static double current(MotorDevice motor, double volts, double speed) {
        var constants = MotorConstants.forKind(motor.kind()).orElseThrow();
        return (volts - speed * 2.0 * Math.PI / constants.raw().speedConstantRadPerSecPerVolt())
                / constants.resistance().in(Units.Ohms);
    }

    @Test
    void simulatedHardwareInversionUsesItsRawShaftFrame() {
        SimulationSession simulation = SimulationSession.create();
        MotorDevice device = MotorDevice.of(MotorKinds.FALCON_500, 31).inverted();
        var motor = simulation.motor(device).state(0.0, -50.0);
        try (HardwareGraph graph = simulation.hardwareGraph()) {
            Output output = new LimitedVoltage(12.0, 40.0);
            OutputApplier.using(graph).apply(new ResolvedOutput(OutputRequest.of(device, output), output));
            var constants = MotorConstants.forKind(MotorKinds.FALCON_500).orElseThrow();
            double backEmf = -50.0 * 2.0 * Math.PI / constants.raw().speedConstantRadPerSecPerVolt();
            assertEquals(-1.0, motor.voltageCommandScale());
            assertEquals(-40.0, (motor.appliedVoltage() - backEmf) / constants.resistance().in(Units.Ohms), 1e-8);
            assertTrue(motor.appliedVoltage() < 0.0);
            assertTrue(device.isInverted());
        }
    }

    @Test
    void promotedInvertedLeaderAndRewiredFollowerUseTheirDeratedCommandHandles() {
        try (FollowerRig rig = new FollowerRig(false, false)) {
            MotorDevice third = MotorDevice.of(MotorKinds.FALCON_500, 34).canbus("rio").follow(rig.leader);
            rig.graph.motor(third);
            RecordingMotor thirdMotor = rig.backend.motors.get(third.id());
            MotorDevice child = MotorDevice.of(MotorKinds.FALCON_500, 38).canbus("third").follow(third).inverted();
            rig.graph.motor(child);
            RecordingMotor childMotor = rig.backend.motors.get(child.id());
            var group = rig.graph.motorGroup(new MotorGroupPolicy(rig.leader, List.of(rig.follower),
                    true, true, 1, 0.5, 1.0, false, 1));
            rig.leaderMotor.speed = 50.0;
            rig.followerMotor.speed = -50.0;
            thirdMotor.speed = 50.0;
            childMotor.speed = -50.0;
            rig.leaderMotor.failed = true;
            rig.graph.refreshInputs();

            assertEquals(rig.follower, group.leader());
            assertEquals(0.5, rig.graph.motor(rig.leader).voltageCommandScale());
            assertEquals(50.0, rig.graph.motor(rig.leader).integratedVelocityRotationsPerSecond());
            assertEquals(1.0, rig.graph.motor(third).voltageCommandScale(),
                    "telemetry retains the original hardware handle");
            assertEquals(0.5, rig.graph.softwareFollowers(rig.leader).get(0).handle().voltageCommandScale(),
                    "commands use the replacement software wrapper and its derate");

            rig.apply(20.0, 40.0, false);

            assertTrue(rig.followerMotor.voltage < 0.0);
            assertTrue(thirdMotor.voltage > 0.0);
            assertEquals(-rig.followerMotor.voltage, thirdMotor.voltage, 1e-9);
            assertEquals(-40.0, rig.current(rig.followerMotor), 1e-8);
            assertEquals(40.0, rig.current(thirdMotor), 1e-8);
            assertEquals(-thirdMotor.voltage, childMotor.voltage, 1e-9);
            assertEquals(-40.0, rig.current(childMotor), 1e-8);
            assertEquals(1, childMotor.directVoltageCommands);
        }
    }

    private record LimitedVoltage(double volts, EffortRequest effort) implements Output.Voltage, EffortCarrier {
        LimitedVoltage(double volts, double amps) { this(volts, new EffortRequest(Units.Amps.of(amps))); }
    }

    private record LimitedVelocity(double velocity, EffortRequest effort) implements Output.Velocity, EffortCarrier { }

    private static final class FollowerRig implements AutoCloseable {
        final MotorDevice leader;
        final MotorDevice follower;
        final FollowerBackend backend = new FollowerBackend();
        final HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend));
        final RecordingMotor leaderMotor;
        final RecordingMotor followerMotor;
        final OutputApplier applier = OutputApplier.using(graph);
        final ControlBinding<?> control;

        FollowerRig(boolean sameBus, boolean leaderInverted) {
            leader = MotorDevice.of(MotorKinds.FALCON_500, 31).canbus("rio").inverted(leaderInverted);
            follower = MotorDevice.of(MotorKinds.FALCON_500, 33).canbus(sameBus ? "rio" : "canivore")
                    .follow(leader).inverted();
            graph.motor(follower);
            leaderMotor = backend.motors.get(leader.id());
            followerMotor = backend.motors.get(follower.id());
            control = Controls.velocity(leader, Units.Rotations).feedback(leader.encoder().measures(Units.Rotations));
        }

        void apply(double volts, double amps, boolean binding) {
            Output output = new LimitedVoltage(volts, amps);
            OutputRequest request = binding ? OutputRequest.of(control, output) : OutputRequest.of(leader, output);
            applier.apply(new ResolvedOutput(request, output));
        }

        double current(RecordingMotor motor) {
            var constants = MotorConstants.forKind(MotorKinds.FALCON_500).orElseThrow();
            return (motor.voltage - motor.speed * 2.0 * Math.PI / constants.raw().speedConstantRadPerSecPerVolt())
                    / constants.resistance().in(Units.Ohms);
        }

        @Override public void close() { graph.close(); }
    }

    private static final class FollowerBackend implements MotorBackend {
        final Map<Integer, RecordingMotor> motors = new HashMap<>();
        @Override public boolean supports(MotorKind kind) { return true; }
        @Override public boolean supportsHardwareFollowing(MotorDevice follower, MotorDevice leader) {
            return follower.canbus().equals(leader.canbus());
        }
        @Override public MotorHandle create(MotorDevice device) {
            RecordingMotor motor = new RecordingMotor(device);
            motors.put(device.id(), motor);
            return motor;
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double voltage;
        private double velocity;
        private double speed;
        private boolean failed;
        private RecordingMotor follows;
        private boolean followInverted;
        private int directVoltageCommands;
        private int velocityCommands;
        private boolean phasedNative;
        private MotorClosedLoopRequest lastRequest;
        private int stops;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public void setVoltage(double volts) { voltage = volts; directVoltageCommands++; }
        @Override public double integratedPositionRotations() { return 0.0; }
        @Override public double integratedVelocityRotationsPerSecond() { return speed; }
        @Override public double appliedVoltage() {
            return follows == null ? voltage : follows.appliedVoltage() * hardwareFollowerVoltageScale();
        }
        @Override public double hardwareFollowerVoltageScale() { return phasedNative || !followInverted ? 1.0 : -1.0; }
        @Override public MotorControlCapabilities controlCapabilities() { return MotorControlCapabilities.voltageClosedLoop(4); }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public double statorCurrentAmps() { return 0.0; }
        @Override public boolean faulted() { return failed; }
        @Override public void follow(MotorHandle leader, boolean inverted) {
            follows = (RecordingMotor) leader;
            followInverted = inverted;
        }
        @Override public void stopFollowing() { follows = null; }
        @Override public void stop() { voltage = 0.0; stops++; }
        @Override public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
            velocityCommands++;
        }
        @Override public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond, MotorClosedLoopRequest request) {
            lastRequest = request;
            setVelocityTargetRotationsPerSecond(rotationsPerSecond);
        }
    }
}
