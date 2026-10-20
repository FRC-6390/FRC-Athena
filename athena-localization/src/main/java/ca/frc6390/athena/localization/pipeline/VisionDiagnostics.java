package ca.frc6390.athena.localization.pipeline;

import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Per-camera frame history for one Kalman localization. Each frame is judged once, by the deepest stage that
 * rejected it, and repeated reports of the same capture timestamp are ignored. Rates are counted over the last
 * second of robot time.
 */
final class VisionDiagnostics {
    static final double WINDOW_SECONDS = 1.0;
    static final double STALE_SECONDS = 0.5;
    static final double FUTURE_TOLERANCE_SECONDS = 0.1;
    private static final int REMEMBERED_FRAMES = 32;

    /** The outcome of one frame at the stage that judged it. */
    record Verdict(boolean accepted, boolean recovering, String reason) {
        static Verdict accept() {
            return new Verdict(true, false, "");
        }

        static Verdict reject(String reason) {
            return new Verdict(false, false, reason == null ? "" : reason);
        }

        static Verdict recovering(String reason) {
            return new Verdict(false, true, reason == null ? "" : reason);
        }
    }

    private static final class Track {
        final String name;
        final ArrayDeque<Double> remembered = new ArrayDeque<>();
        final ArrayDeque<Double> accepted = new ArrayDeque<>();
        final ArrayDeque<Double> rejected = new ArrayDeque<>();
        double newestFrame = Double.NaN;
        Verdict newestVerdict = Verdict.reject("");
        int newestTagCount;
        double lastAccepted = Double.NaN;
        /** Camera clock, when the source can report whether it is synced to robot time. */
        ca.frc6390.athena.vision.device.CameraClock clock;

        Track(String name) {
            this.name = name;
        }
    }

    private final Map<String, Track> tracks = new LinkedHashMap<>();

    /**
     * Records the verdict for one frame. A frame whose capture timestamp was already recorded for this source is
     * ignored, so re-reported frames do not count twice.
     */
    void observe(Object source, double frameTimestampSeconds, double nowSeconds, Verdict verdict, int tagCount) {
        Track track = tracks.computeIfAbsent(sourceName(source), Track::new);
        if (source instanceof ca.frc6390.athena.vision.device.CameraClock clock) {
            track.clock = clock;
        }
        for (Double seen : track.remembered) {
            if (seen == frameTimestampSeconds) {
                return;
            }
        }
        track.remembered.addLast(frameTimestampSeconds);
        if (track.remembered.size() > REMEMBERED_FRAMES) {
            track.remembered.removeFirst();
        }
        // "Newest" is the most recently received frame, not the largest timestamp: a frame stamped in the
        // future must not pin its camera to UNSYNCED after the clock syncs and correct frames arrive.
        track.newestFrame = frameTimestampSeconds;
        track.newestVerdict = verdict;
        track.newestTagCount = tagCount;
        if (verdict.accepted()) {
            track.accepted.addLast(nowSeconds);
            track.lastAccepted = nowSeconds;
        } else {
            track.rejected.addLast(nowSeconds);
        }
    }

    /** Current status of every source seen so far, judged at robot time {@code nowSeconds}. */
    List<VisionSourceStatus> snapshot(double nowSeconds) {
        List<VisionSourceStatus> out = new ArrayList<>(tracks.size());
        for (Track track : tracks.values()) {
            prune(track.accepted, nowSeconds);
            prune(track.rejected, nowSeconds);
            double age = Double.isFinite(nowSeconds) ? nowSeconds - track.newestFrame : Double.NaN;
            VisionSourceStatus.State state;
            String reason;
            if (Double.isFinite(age) && -age > FUTURE_TOLERANCE_SECONDS) {
                state = VisionSourceStatus.State.UNSYNCED;
                reason = LocalizationFilters.format("frame %.1f s in the future", -age);
            } else if (Double.isFinite(age) && age > STALE_SECONDS) {
                state = VisionSourceStatus.State.STALE;
                reason = LocalizationFilters.format("no frame for %.1f s", age);
            } else if (track.newestVerdict.accepted()) {
                state = VisionSourceStatus.State.OK;
                reason = "";
            } else {
                state = track.newestVerdict.recovering()
                        ? VisionSourceStatus.State.RECOVERING
                        : VisionSourceStatus.State.REJECTED;
                reason = track.newestVerdict.reason();
            }
            double since = Double.isFinite(nowSeconds) && !Double.isNaN(track.lastAccepted)
                    ? nowSeconds - track.lastAccepted
                    : Double.NaN;
            out.add(new VisionSourceStatus(
                    track.name,
                    state,
                    reason,
                    Hertz.of(track.accepted.size() / WINDOW_SECONDS),
                    Hertz.of(track.rejected.size() / WINDOW_SECONDS),
                    Seconds.of(age),
                    track.newestTagCount,
                    Seconds.of(since),
                    timeSync(track)));
        }
        return out;
    }

    private static Boolean timeSync(Track track) {
        if (track.clock == null) {
            return null;
        }
        ca.frc6390.athena.vision.device.CameraClockStatus status = track.clock.clockStatus();
        return status == null || status.isUnknown() ? null : status.synced();
    }

    private static void prune(ArrayDeque<Double> events, double nowSeconds) {
        if (!Double.isFinite(nowSeconds)) {
            return;
        }
        Iterator<Double> iterator = events.iterator();
        while (iterator.hasNext()) {
            if (nowSeconds - iterator.next() >= WINDOW_SECONDS) {
                iterator.remove();
            }
        }
    }

    static String sourceName(Object source) {
        if (source instanceof ca.frc6390.athena.vision.device.CameraDevice camera
                && camera.name() != null && !camera.name().isBlank()) {
            return camera.name();
        }
        if (source instanceof String text && !text.isBlank()) {
            return text;
        }
        if (source instanceof Localization localization && !localization.debugName().isBlank()) {
            return localization.debugName();
        }
        if (source == null) {
            return "unknown";
        }
        return source.getClass().getSimpleName() + "-" + Integer.toHexString(System.identityHashCode(source));
    }
}
