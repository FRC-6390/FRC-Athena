package ca.frc6390.athena.robot.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.RecordMetadata;
import ca.frc6390.athena.mechanism.core.RuntimeMarkers;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Mechanism-side markers (RuntimeMarkers SPI) land in the replay recording while replay is enabled. */
class ReplayRuntimeMarkersTest {
    @TempDir
    Path directory;

    @Test
    void mechanismMarkersAndKeyframeRequestsReachTheRecording() throws IOException {
        SimulationSession session = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(session);
        runtime.replay(ReplayPolicy.of(), directory);
        ReplayRecorder recorder = runtime.replayRecorder().orElseThrow();
        assertInstanceOf(ReplayRuntimeMarkers.class, RuntimeMarkers.current());

        RecordMetadata[] metadata = new RecordMetadata[1];
        long[] lastSequenceAtMetadata = new long[1];
        long[] lastSequenceAtCapture = new long[1];
        for (int cycle = 0; cycle < 40; cycle++) {
            double now = cycle * 0.02;
            if (cycle == 5) RuntimeMarkers.current().mark(RuntimeMarkers.CHECK, "rejected: locked (Locks.InMatch)");
            if (cycle == 10) {
                metadata[0] = RuntimeMarkers.current().recordMetadata();
                lastSequenceAtMetadata[0] = recorder.currentFrameIndex();
            }
            if (cycle == 20) RuntimeMarkers.current().mark(RuntimeMarkers.STAGE_CHANGE, "stage NORMAL -> HOT");
            if (cycle == 25) {
                RuntimeMarkers.current().requestFullCapture();
                lastSequenceAtCapture[0] = recorder.currentFrameIndex();
            }
            runtime.periodic(
                    new MechanismContext(now, 0.0, 0.02, true, false, true),
                    new EventContext(now, 0.02, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
            runtime.simulationPeriodic(now, 0.02);
        }
        Path file = ReplayRecorderTest.flushed(recorder);

        // Metadata: no FMS match, replay id is the session, frame index is the latest recorded sequence.
        assertEquals(recorder.sessionId(), metadata[0].replayId().orElseThrow());
        assertEquals(lastSequenceAtMetadata[0], metadata[0].frameIndex().orElseThrow());
        assertTrue(lastSequenceAtMetadata[0] > 0);
        assertEquals("", metadata[0].eventName());
        assertEquals(0, metadata[0].matchNumber());

        List<Integer> kinds = new ArrayList<>();
        List<String> messages = new ArrayList<>();
        int keyframesAfterRequest = 0;
        try (ReplayReader reader = ReplayReader.open(file)) {
            ReplayReader.Cursor cursor = reader.cursor();
            long previousSequence = -1;
            while (cursor.next()) {
                if (cursor.isFrame()) {
                    if (cursor.sequence() == lastSequenceAtCapture[0] + 1 && cursor.keyframe()) keyframesAfterRequest++;
                    previousSequence = cursor.sequence();
                } else {
                    kinds.add(cursor.markerKind());
                    messages.add(cursor.markerMessage());
                }
            }
            assertTrue(previousSequence >= 80, "two frames per cycle: " + previousSequence);
        }
        assertTrue(kinds.contains(RuntimeMarkers.CHECK), "CHECK marker: " + kinds);
        assertTrue(kinds.contains(RuntimeMarkers.STAGE_CHANGE), "STAGE_CHANGE marker: " + kinds);
        assertTrue(messages.contains("rejected: locked (Locks.InMatch)"));
        assertEquals(1, keyframesAfterRequest, "requestFullCapture forces a keyframe on the next cycle");

        runtime.close();
        assertFalse(RuntimeMarkers.current() instanceof ReplayRuntimeMarkers, "markers uninstalled on close");
    }
}
