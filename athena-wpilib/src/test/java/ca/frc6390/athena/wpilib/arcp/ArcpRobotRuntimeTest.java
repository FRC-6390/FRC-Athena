package ca.frc6390.athena.wpilib.arcp;

import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.arcp.ArcpDictEntry;
import ca.frc6390.athena.arcp.ArcpPolicy;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.TelemetryAction;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.mechanism.staging.Staging;
import ca.frc6390.athena.mechanism.persist.Locks;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import ca.frc6390.athena.mechanism.persist.TuningLocks;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.ReplayRecorder;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.runtime.wire.Wire;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.net.DatagramSocket;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Starts a simulated robot runtime with the ARCP bridge on ephemeral ports and talks to it with the test client.
 * Covers the dictionary, cycle values, lock signals, writes (including FMS-locked writes using the simulated driver
 * station), actions, file service, tuning layers and export files, and replay streaming with markers and backfill.
 */
class ArcpRobotRuntimeTest {
    private static final String GAIN = "arcpTestMechanism/Values/gain";
    private static final String READY = "arcpTestMechanism/Values/status/ready";
    private static final String RUN = "arcpTestMechanism/Actions/run";
    private static final String POWERED = "arcpPoweredMechanism";

    @TempDir
    Path temp;

    private RobotRuntime runtime;
    private ArcpBridge bridge;
    private ArcpTestClient client;
    private DatagramSocket udp;
    private double now;

    @BeforeAll
    static void initHal() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
    }

    @BeforeEach
    void setFmsDetached() {
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        now = 0.0;
    }

    @AfterEach
    void tearDown() throws Exception {
        if (client != null) client.close();
        if (udp != null) udp.close();
        if (bridge != null) bridge.close();
        if (runtime != null) runtime.close();
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void dictionaryAndCycleValuesReachSubscribers() throws Exception {
        start(ArcpPolicy.of().controlPort(0).realtimePort(0), null, null);
        assertTrue(client.dictionary().containsKey(GAIN));
        ArcpTestClient.Dict gain = client.signal(GAIN);
        assertTrue((gain.flags() & ArcpDictEntry.FLAG_WRITABLE) != 0);
        assertTrue((gain.flags() & ArcpDictEntry.FLAG_PERSISTED) != 0);
        assertEquals(Locks.InMatch.class.getName(), gain.lockGroup());
        assertEquals(ArcpDictEntry.ROLE_CONTROL, gain.role());
        assertEquals(ArcpDictEntry.ROLE_ACTION, client.signal(RUN).role());
        ArcpTestClient.Dict lock = client.signal("athena/locks/" + Locks.InMatch.class.getName());
        assertEquals(ArcpDictEntry.ROLE_LOCK, lock.role());

        subscribeAll();
        cycles(3);
        List<ArcpTestClient.Frame> frames = ArcpTestClient.receiveFrames(udp, 500);
        Map<Long, Object> values = merged(frames);
        assertEquals(2.0, (Double) values.get(gain.id()), 1e-9);
        assertEquals(Boolean.TRUE, values.get(client.signal(READY).id()));
        assertEquals(Boolean.FALSE, values.get(lock.id()));
    }

    @Test
    void robotSystemSignalsArePublishedUnderTheDashboardPaths() throws Exception {
        start(ArcpPolicy.of().controlPort(0).realtimePort(0), null, null);
        for (String path : List.of("athena/robot/mode", "athena/robot/matchTime", "athena/robot/alliance",
                "athena/power/battery/voltage", "athena/power/bus/demandAmps", "athena/power/bus/shedding")) {
            ArcpTestClient.Dict dict = client.signal(path);
            assertEquals(ArcpDictEntry.ROLE_SYSTEM, dict.role(), path);
            assertEquals(0, dict.flags() & ArcpDictEntry.FLAG_WRITABLE, path);
        }
        subscribeAll();
        cycles(3);
        Map<Long, Object> values = merged(ArcpTestClient.receiveFrames(udp, 500));
        assertEquals("Disabled", values.get(client.signal("athena/robot/mode").id()));
        assertTrue((Double) values.get(client.signal("athena/power/battery/voltage").id()) > 0.0);
    }

    @Test
    void writesGoThroughTelemetryAndFmsLocksRejectThem() throws Exception {
        start(ArcpPolicy.of().controlPort(0).realtimePort(0), null, null);
        long gain = client.signal(GAIN).id();

        ArcpTestClient.WriteAnswer accepted = write(gain, 3.5);
        assertTrue(accepted.accepted(), accepted.reason());
        assertEquals(3.5, (Double) accepted.actualValue(), 1e-9);
        assertEquals(3.5, schemaGain(), 1e-9);

        ArcpTestClient.WriteAnswer wrongType = write(gain, "fast");
        assertFalse(wrongType.accepted());
        assertTrue(wrongType.reason().contains("type mismatch"), wrongType.reason());

        ArcpTestClient.WriteAnswer readOnly = write(client.signal(READY).id(), true);
        assertFalse(readOnly.accepted());
        assertEquals("rejected: read-only", readOnly.reason());

        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        ArcpTestClient.WriteAnswer locked = write(gain, 4.0);
        assertFalse(locked.accepted());
        assertEquals("rejected: locked (InMatch)", locked.reason());
        assertEquals(3.5, (Double) locked.actualValue(), 1e-9);
        assertEquals(3.5, schemaGain(), 1e-9);

        subscribeAll();
        cycles(3);
        Map<Long, Object> values = merged(ArcpTestClient.receiveFrames(udp, 500));
        assertEquals(Boolean.TRUE, values.get(client.signal("athena/locks/" + Locks.InMatch.class.getName()).id()));
    }

    @Test
    void actionRunAndCancelUseTheMechanismAction() throws Exception {
        start(ArcpPolicy.of().controlPort(0).realtimePort(0), null, null);
        long run = client.signal(RUN).id();
        TelemetryAction action = schemaAction("run");
        assertFalse(action.running());

        int id = client.sendAction(run);
        ArcpTestClient.Reply reply = pump(() -> client.awaitReply(id));
        assertEquals(ArcpTestClient.STATUS_OK, reply.status(), reply.reason());
        assertTrue(action.running());

        int cancel = client.sendActionCancel(run);
        ArcpTestClient.Reply cancelled = pump(() -> client.awaitReply(cancel));
        assertEquals(ArcpTestClient.STATUS_OK, cancelled.status(), cancelled.reason());
        assertFalse(action.running());
    }

    @Test
    void fileServiceListsDownloadsAndDeletesReplaysAndExposesTuningFiles() throws Exception {
        Path replays = temp.resolve("replays");
        Files.createDirectories(replays);
        Files.write(replays.resolve("match-1.athrp"), "replay-bytes".getBytes(StandardCharsets.UTF_8));
        runtime = RobotRuntime.simulated(SimulationSession.create());
        runtime.replay(ReplayPolicy.of().directory(replays), replays);
        PersistentStore store = PersistentStore.open(temp.resolve("store"));
        store.put(GAIN, 3.5);
        start(runtime, ArcpPolicy.of().controlPort(0).realtimePort(0), store);
        cycles(3);

        ArcpTestClient.Reply list = client.fileList();
        assertEquals(ArcpTestClient.STATUS_OK, list.status(), list.reason());
        assertTrue(((List<?>) list.data()).contains("replays/match-1.athrp"), list.data().toString());

        ArcpTestClient.Reply download = client.fileDownload("replays/match-1.athrp");
        assertEquals(ArcpTestClient.STATUS_OK, download.status());
        assertArrayEquals("replay-bytes".getBytes(StandardCharsets.UTF_8), (byte[]) ((Object[]) download.data())[1]);

        ArcpTestClient.Reply diff = client.fileDownload("tuning/diff.txt");
        assertEquals(ArcpTestClient.STATUS_OK, diff.status(), diff.reason());
        String diffText = text(diff);
        assertTrue(diffText.contains(GAIN + "\t3.5\t-\t2.0\trobot-changed"), diffText);

        ArcpTestClient.Reply pull = client.fileDownload("tuning/pull.txt");
        assertTrue(text(pull).contains(GAIN + "\tD\t3.5"), text(pull));

        ArcpTestClient.Reply status = client.fileDownload("status/status.txt");
        assertTrue(text(status).contains("replay.streaming=false"), text(status));

        ArcpTestClient.Reply deleted = client.fileDelete("replays/match-1.athrp");
        assertEquals(ArcpTestClient.STATUS_OK, deleted.status(), deleted.reason());
        assertFalse(Files.exists(replays.resolve("match-1.athrp")));
    }

    @Test
    void deployedTuningSitsBetweenCodeDefaultAndRobotStore() throws Exception {
        Path deployed = temp.resolve("deploy/athena/tuning");
        Files.createDirectories(deployed.getParent());
        Files.writeString(deployed, "# athena persistent store v1\n" + GAIN + "\tD\t2.75\n");
        PersistentStore store = PersistentStore.open(temp.resolve("store"));
        store.fallback(PersistentStore.readTuning(deployed));
        runtime = RobotRuntime.simulated(SimulationSession.create());
        TuningLocks locks = tuningLocks();
        runtime.tuningLocks(locks);
        runtime.persistentStore(store, null);
        mechanism = new ArcpTestMechanism();
        runtime.register(mechanism);
        assertEquals(2.75, schemaGain(), 1e-9, "deployed file overrides the code default");
        assertEquals(2.0, runtime.telemetryPersistence().codeDefaults().get(GAIN));

        bridge = ArcpBridge.attach(ArcpPolicy.of().controlPort(0).realtimePort(0).deployedTuningFile(deployed), runtime, null, locks::isLocked,
                store, true);
        client = new ArcpTestClient(bridge.controlPort());
        assertEquals(ArcpTestClient.STATUS_OK, client.hello(0).status());
        cycles(3);
        ArcpTestClient.Reply diffReply = client.fileDownload("tuning/diff.txt");
        assertEquals(ArcpTestClient.STATUS_OK, diffReply.status(), diffReply.reason());
        String diff = text(diffReply);
        assertTrue(diff.contains(GAIN + "\t2.75\t2.75\t2.0\tdeployed"), diff);
        assertFalse(store.keys().contains(GAIN), "deployed values must not be written into the robot store");
    }

    @Test
    void controlSetpointsMotorThermalAndStageAreInTheDictionaryAndCarryValues() throws Exception {
        runtime = RobotRuntime.simulated(SimulationSession.create());
        ArcpPoweredMechanism powered = new ArcpPoweredMechanism();
        runtime.register(powered);
        bridge = ArcpBridge.attach(ArcpPolicy.of().controlPort(0).realtimePort(0), runtime, null, l -> false, null, true);
        client = new ArcpTestClient(bridge.controlPort());
        udp = new DatagramSocket(0);
        assertEquals(ArcpTestClient.STATUS_OK, client.hello(udp.getLocalPort()).status());

        String control = POWERED + "/Controls/control/State/";
        String motor = POWERED + "/Devices/motor/State/";
        for (String path : List.of(control + "Target", control + "Reference", control + "ReferenceVelocity",
                control + "Error", control + "Output", motor + "TemperatureCelsius", motor + "Derate",
                motor + "PowerCapAmps", POWERED + "/State/Stage")) {
            ArcpTestClient.Dict dict = client.signal(path);
            assertEquals(ArcpDictEntry.ROLE_STATE, dict.role(), path);
            assertEquals(0, dict.flags() & ArcpDictEntry.FLAG_WRITABLE, path);
        }
        assertEquals("V", client.signal(control + "Output").unit());
        assertEquals("C", client.signal(motor + "TemperatureCelsius").unit());
        assertEquals("A", client.signal(motor + "PowerCapAmps").unit());

        runtime.request(powered.spin);
        subscribeAll();
        for (int i = 0; i < 5; i++) {
            runtime.teleopPeriodic(now += 0.02, 0.02);
            bridge.periodic(now);
        }
        Map<Long, Object> values = merged(ArcpTestClient.receiveFrames(udp, 500));
        assertEquals(5.0, (Double) values.get(client.signal(control + "Target").id()), 1e-9);
        assertTrue(values.containsKey(client.signal(control + "Output").id()), "applied volts are sent");
        assertEquals("NORMAL", values.get(client.signal(POWERED + "/State/Stage").id()));
        assertTrue(values.containsKey(client.signal(motor + "TemperatureCelsius").id()));
    }

    @Test
    void powerMotorChannelsJoinTheDictionaryAfterTheFirstPowerCycleWithoutRenumbering() throws Exception {
        runtime = RobotRuntime.simulated(SimulationSession.create());
        ArcpPoweredMechanism powered = new ArcpPoweredMechanism();
        runtime.register(powered);
        bridge = ArcpBridge.attach(ArcpPolicy.of().controlPort(0).realtimePort(0), runtime, null, l -> false, null, true);
        client = new ArcpTestClient(bridge.controlPort());
        udp = new DatagramSocket(0);
        assertEquals(ArcpTestClient.STATUS_OK, client.hello(udp.getLocalPort()).status());

        String current = "athena/power/" + powered.motor.defaultName() + "/current";
        String cap = "athena/power/" + powered.motor.defaultName() + "/capAmps";
        assertFalse(client.dictionary().containsKey(current), "no power motors before the first cycle");
        long targetId = client.signal(POWERED + "/Controls/control/State/Target").id();

        for (int i = 0; i < 60; i++) {
            runtime.teleopPeriodic(now += 0.02, 0.02);
            bridge.periodic(now);
        }
        // Any request reads the pending schema push, which the republished dictionary arrives in.
        ArcpTestClient.Reply list = pump(() -> client.fileList());
        assertEquals(ArcpTestClient.STATUS_OK, list.status(), list.reason());
        assertTrue(client.dictionary().containsKey(current), "the power channels appear once the motor is known");
        assertTrue(client.dictionary().containsKey(cap));
        assertEquals("A", client.signal(current).unit());
        assertEquals(targetId, client.signal(POWERED + "/Controls/control/State/Target").id(),
                "existing ids do not change when the dictionary grows");

        subscribeAll();
        for (int i = 0; i < 3; i++) {
            runtime.teleopPeriodic(now += 0.02, 0.02);
            bridge.periodic(now);
        }
        Map<Long, Object> values = merged(ArcpTestClient.receiveFrames(udp, 500));
        assertTrue(values.containsKey(client.signal(current).id()), "the current is sent");
        assertEquals(0.0, (Double) values.get(client.signal(cap).id()), 1e-9, "an uncapped motor reports 0 A");
    }

    @Test
    void replayRecordsStreamOnlyToOptedInSessionsMarkersAreForwardedAndBackfillIsServed() throws Exception {
        Path replays = temp.resolve("replays");
        runtime = RobotRuntime.simulated(SimulationSession.create());
        runtime.replay(ReplayPolicy.of().stream(true).directory(replays), replays);
        start(runtime, ArcpPolicy.of().controlPort(0).realtimePort(0), null);
        ReplayRecorder recorder = runtime.replayRecorder().orElseThrow();
        assertTrue(recorder.streaming());

        ArcpTestClient.Reply optIn = replayOptIn();
        assertEquals(ArcpTestClient.STATUS_OK, optIn.status(), optIn.reason());
        for (int i = 0; i < 5; i++) {
            runtime.teleopPeriodic(now += 0.02, 0.02);
        }
        cycles(3);
        List<ArcpTestClient.WireRecord> records = ArcpTestClient.receiveRecords(udp, 500);
        assertTrue(records.stream().anyMatch(r -> r.type() == Wire.RECORD_FRAME), "no replay frames streamed");
        assertTrue(records.stream().anyMatch(r -> r.type() == 0x01), "no replay dictionary streamed " + records.size());

        recorder.mark(Wire.MARKER_FAULT, "boom");
        List<ArcpTestClient.Marker> markers = ArcpTestClient.receiveMarkers(udp, 500);
        assertTrue(markers.stream().anyMatch(m -> m.kind() == Wire.MARKER_FAULT && "boom".equals(m.message())),
                markers.toString());

        ArcpTestClient.Reply backfill = pump(() -> client.backfill(0, Long.MAX_VALUE));
        assertEquals(ArcpTestClient.STATUS_OK, backfill.status(), backfill.reason());
        assertTrue(((byte[]) backfill.data()).length > 0, "backfill returned nothing");
    }

    @Test
    void takenRealtimePortFallsBackInSimulationAndFailsOnRobot() throws Exception {
        runtime = RobotRuntime.simulated(SimulationSession.create());
        try (DatagramSocket taken = new DatagramSocket(0)) {
            ArcpPolicy policy = ArcpPolicy.of().controlPort(0).realtimePort(taken.getLocalPort());
            ArcpBridge simBridge = ArcpBridge.attach(policy, runtime, null, l -> false, null, true);
            try {
                assertTrue(simBridge.enabled(), "simulation falls back to ephemeral ports");
                assertTrue(simBridge.controlPort() > 0);
            } finally {
                simBridge.close();
            }

            ArcpBridge robotBridge = ArcpBridge.attach(policy, runtime, null, l -> false, null, false);
            try {
                assertFalse(robotBridge.enabled(), "a robot with a taken realtime port fails ARCP loudly");
            } finally {
                robotBridge.close();
            }
        }
    }

    @Test
    void disabledPolicyStartsNothing() {
        runtime = RobotRuntime.simulated(SimulationSession.create());
        bridge = ArcpBridge.attach(ArcpPolicy.disabled(), runtime, null, l -> false, null, true);
        assertFalse(bridge.enabled());
        assertEquals(-1, bridge.controlPort());
        bridge.periodic(0.0);
    }

    // --- helpers -----------------------------------------------------------------------------------------

    private ArcpTestMechanism mechanism;

    private static TuningLocks tuningLocks() {
        return new TuningLocks().define(Locks.InMatch.class, DriverStation::isFMSAttached);
    }

    private void start(ArcpPolicy policy, Object unused, PersistentStore store) throws Exception {
        if (runtime == null) {
            runtime = RobotRuntime.simulated(SimulationSession.create());
        }
        start(runtime, policy, store);
    }

    private void start(RobotRuntime runtime, ArcpPolicy policy, PersistentStore store) throws Exception {
        TuningLocks locks = tuningLocks();
        runtime.tuningLocks(locks);
        if (store != null) {
            runtime.persistentStore(store, null);
        }
        mechanism = new ArcpTestMechanism();
        runtime.register(mechanism);
        ReplayRecorder recorder = runtime.replayRecorder().orElse(null);
        bridge = ArcpBridge.attach(policy, runtime, recorder, locks::isLocked, store, true);
        assertTrue(bridge.enabled());
        client = new ArcpTestClient(bridge.controlPort());
        udp = new DatagramSocket(0);
        assertEquals(ArcpTestClient.STATUS_OK, client.hello(udp.getLocalPort()).status());
    }

    private void subscribeAll() throws Exception {
        assertEquals(ArcpTestClient.STATUS_OK, client.subscribe("**").status());
    }

    private ArcpTestClient.Reply replayOptIn() throws Exception {
        return pump(() -> client.replaySubscribe());
    }

    private ArcpTestClient.WriteAnswer write(long signal, Object value) throws Exception {
        int id = client.sendWrite(signal, value);
        return pump(() -> client.awaitWrite(id));
    }

    private double schemaGain() {
        return runtime.mechanismTelemetrySchema().values().get(GAIN).number();
    }

    private TelemetryAction schemaAction(String name) {
        return runtime.mechanismTelemetrySchema().find("arcpTestMechanism/Actions").orElseThrow()
                .actions().get(name);
    }

    /** Runs robot cycles on the calling thread. */
    private void cycles(int count) {
        for (int i = 0; i < count; i++) {
            bridge.periodic(now += 0.02);
        }
    }

    /** Runs the robot side of the bridge while the test waits for a client answer. */
    private <T> T pump(Callable<T> call) throws Exception {
        AtomicBoolean done = new AtomicBoolean();
        Thread robot = new Thread(() -> {
            while (!done.get()) {
                bridge.periodic(now += 0.02);
                try {
                    Thread.sleep(2);
                } catch (InterruptedException interrupted) {
                    return;
                }
            }
        });
        robot.start();
        try {
            return call.call();
        } finally {
            done.set(true);
            robot.join();
        }
    }

    private static Map<Long, Object> merged(List<ArcpTestClient.Frame> frames) {
        java.util.HashMap<Long, Object> out = new java.util.HashMap<>();
        for (ArcpTestClient.Frame frame : frames) {
            out.putAll(frame.values());
        }
        return out;
    }

    private static String text(ArcpTestClient.Reply reply) {
        return new String(((byte[]) ((Object[]) reply.data())[1]), StandardCharsets.UTF_8);
    }

    /** Test robot mechanism with a velocity loop on one motor and a two-stage staging. */
    static final class ArcpPoweredMechanism implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 21);
        final ControlBinding<?> control = Controls.velocity(motor, Units.Rotations).pid(1.0, 0.0, 0.0);
        public final Action spin = control.velocity(Units.RotationsPerSecond.of(5.0));
        public final Staging<Stage> health = Staging.of(Stage.NORMAL)
                .stage(Stage.HOT, () -> false, control.limitEffort(0.5));

        enum Stage { NORMAL, HOT }
    }

    /** Test robot mechanism: one persisted writable number, a read-only status and one action. */
    static final class ArcpTestMechanism implements Mechanism {
        @Telemetry(writable = true, min = 0.0, max = 10.0, persist = true)
        double gain = 2.0;
        public final Action run = Actions.waitSeconds(Seconds.of(1.0));

        @Telemetry("status/ready")
        boolean ready() {
            return true;
        }
    }
}
