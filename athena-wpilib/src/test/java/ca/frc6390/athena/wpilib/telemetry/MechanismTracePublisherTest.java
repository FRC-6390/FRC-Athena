package ca.frc6390.athena.wpilib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.MechanismTraceSnapshot;
import ca.frc6390.athena.mechanism.core.MechanismTraceLevel;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class MechanismTracePublisherTest {
    @Test
    void liveProfileOverrideIsCaseInsensitiveAndRejectsUnknownValues() {
        assertEquals(MechanismTracePublisher.Profile.CAPTURE,
                MechanismTracePublisher.parseProfile(" capture ", MechanismTracePublisher.Profile.SUMMARY));
        assertEquals(MechanismTracePublisher.Profile.OFF,
                MechanismTracePublisher.parseProfile("OFF", MechanismTracePublisher.Profile.SUMMARY));
        assertEquals(MechanismTracePublisher.Profile.SUMMARY,
                MechanismTracePublisher.parseProfile("unknown", MechanismTracePublisher.Profile.SUMMARY));
        NetworkTableInstance instance = NetworkTableInstance.create();
        try (MechanismTracePublisher publisher = new MechanismTracePublisher(instance)) {
            assertEquals(MechanismTraceLevel.SUMMARY, publisher.traceLevel());
            assertEquals(0.10, publisher.runtimePeriodSeconds(), 1.0e-9);
            assertEquals(0.02, publisher.profile(MechanismTracePublisher.Profile.CAPTURE)
                    .runtimePeriodSeconds(), 1.0e-9);
            assertEquals(MechanismTraceLevel.OFF,
                    publisher.profile(MechanismTracePublisher.Profile.OFF).traceLevel());
        } finally {
            instance.close();
        }
    }

    @Test
    void summaryProfilePublishesAtTenHertz() {
        NetworkTableInstance nt = NetworkTableInstance.create();
        String statePath = "/Athena/Mechanisms/robot/Trace/State";
        try (MechanismTracePublisher publisher = new MechanismTracePublisher(nt)
                .profile(MechanismTracePublisher.Profile.SUMMARY)) {
            publisher.publish(List.of(snapshot(1.0)));
            long firstChange = nt.getEntry(statePath).getLastChange();
            publisher.publish(List.of(snapshot(1.05)));
            assertEquals(firstChange, nt.getEntry(statePath).getLastChange());
            publisher.publish(List.of(snapshot(1.11)));
            assertTrue(nt.getEntry(statePath).getLastChange() > firstChange);
        } finally {
            nt.close();
        }
    }

    @Test
    void captureProfilePublishesPackedFramesAndMetadataOverNt4() {
        NetworkTableInstance nt = NetworkTableInstance.create();
        nt.startLocal();
        String root = "/Athena/Mechanisms/robot/Trace";
        try (var state = nt.getStructTopic(root + "/State", MechanismTracePublisher.StateFrame.STRUCT)
                        .subscribe(new MechanismTracePublisher.StateFrame(0, 0, -1, 0, false, false));
                var controls = nt.getStructArrayTopic(root + "/Controls", MechanismTracePublisher.ControlFrame.STRUCT)
                        .subscribe(new MechanismTracePublisher.ControlFrame[0]);
                var metadata = nt.getStringArrayTopic(root + "/Metadata").subscribe(new String[0]);
                MechanismTracePublisher publisher = new MechanismTracePublisher(nt)
                        .profile(MechanismTracePublisher.Profile.CAPTURE)) {
            publisher.publish(List.of(snapshot()));
            nt.flushLocal();

            assertEquals(3, state.get().schedulerStep());
            assertEquals(1, controls.get().length);
            assertEquals(1.0, controls.get()[0].proportionalVolts(), 1.0e-9);
            assertTrue(Arrays.stream(metadata.get()).anyMatch(value -> value.endsWith("=turret")));
        } finally {
            nt.stopLocal();
            nt.close();
        }
    }

    @Test
    void captureFramesPreserveMechanismTraceValues() {
        MechanismTraceSnapshot snapshot = snapshot();

        assertEquals(3, MechanismTracePublisher.stateFrame(snapshot).schedulerStep());
        assertEquals(5.0, MechanismTracePublisher.controlFrame(snapshot.controls().get(0)).goal(), 1.0e-9);
        assertEquals(1.0,
                MechanismTracePublisher.controlFrame(snapshot.controls().get(0)).proportionalVolts(), 1.0e-9);
        assertEquals(7.0,
                MechanismTracePublisher.motorFrame(snapshot.motors().get(0)).supplyCurrentAmps(), 1.0e-9);
        assertEquals(true, MechanismTracePublisher.candidateFrame(snapshot.candidates().get(0)).selected());
        assertEquals(true, MechanismTracePublisher.hookFrame(snapshot.hooks().get(0)).triggeredThisCycle());
    }

    @Test
    void everyWireStructPacksExactlyItsDeclaredSize() {
        assertPackSize(MechanismTracePublisher.StateFrame.STRUCT,
                new MechanismTracePublisher.StateFrame(1, 2, 3, 4, true, false));
        assertPackSize(MechanismTracePublisher.CandidateFrame.STRUCT,
                new MechanismTracePublisher.CandidateFrame(1, 2, 3, 4, 5, (byte) 1, (byte) 2, true));
        assertPackSize(MechanismTracePublisher.ControlFrame.STRUCT,
                new MechanismTracePublisher.ControlFrame(
                        1, (byte) 2, (byte) 3, (byte) 4, 5,
                        6, 7, 8, 9, 10, 11, 12, 13, 14,
                        15, 16, 17, 18, 19, 20, 21, 22));
        assertPackSize(MechanismTracePublisher.MotorFrame.STRUCT,
                new MechanismTracePublisher.MotorFrame(1, (byte) 2, 3, 4, 5, 6, 7, 8));
        assertPackSize(MechanismTracePublisher.HookFrame.STRUCT,
                new MechanismTracePublisher.HookFrame(1, true, false, true));
    }

    private static MechanismTraceSnapshot snapshot() {
        return snapshot(1.5);
    }

    private static MechanismTraceSnapshot snapshot(double timestampSeconds) {
        return new MechanismTraceSnapshot(
                "robot",
                timestampSeconds,
                0.75,
                true,
                "shoot",
                "Sequence",
                "ControlPosition",
                3,
                false,
                1,
                List.of(new MechanismTraceSnapshot.ActionCandidate(
                        "lease", "ControlPosition", 8, 0, true, "position", 6.0, List.of("turret"))),
                List.of(new MechanismTraceSnapshot.Control(
                        "turret", "position", 6.0, 6.0, 5.0,
                        4.0, 1.0, 2.0, 3.0, 0.5, 3.0,
                        1.0, 0.25, 0.1, 0.2, 0.3, 0.4, 0.5,
                        "voltage", 3.5, "OPEN_LOOP", true, false, false)),
                List.of(new MechanismTraceSnapshot.Motor(
                        "turret", "voltage", 3.5, 3.0, 0.5, 3.5, 7.0, 8.0)),
                List.of(new MechanismTraceSnapshot.Hook("home", true, true, true)));
    }

    private static <T> void assertPackSize(edu.wpi.first.util.struct.Struct<T> struct, T value) {
        ByteBuffer buffer = ByteBuffer.allocate(struct.getSize()).order(ByteOrder.LITTLE_ENDIAN);
        struct.pack(buffer, value);
        assertEquals(struct.getSize(), buffer.position());
    }
}
