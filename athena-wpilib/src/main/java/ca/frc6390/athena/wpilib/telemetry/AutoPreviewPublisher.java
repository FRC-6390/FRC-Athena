package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.auto.AutoPreview;
import ca.frc6390.athena.auto.PathPreview;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Publishes a selected auto's Action plan and AdvantageScope-compatible path geometry. */
public final class AutoPreviewPublisher implements AutoCloseable {
    private static final String ROOT = "/Athena/Auto";
    private final StringPublisher selected;
    private final StringArrayPublisher plan;
    private final StringArrayPublisher pathKeys;
    private final StructArrayPublisher<Pose2d> combinedPath;
    private final StructArrayPublisher<Pose2d> eventPoses;
    private final StringArrayPublisher eventLabels;
    private final NetworkTableInstance networkTables;
    private final Map<String, StructArrayPublisher<Pose2d>> pathPublishers = new LinkedHashMap<>();
    private String lastSignature = "";

    public AutoPreviewPublisher() { this(NetworkTableInstance.getDefault()); }

    AutoPreviewPublisher(NetworkTableInstance networkTables) {
        this.networkTables = Objects.requireNonNull(networkTables, "networkTables");
        selected = networkTables.getStringTopic(ROOT + "/Selected").publish();
        plan = networkTables.getStringArrayTopic(ROOT + "/Plan").publish();
        pathKeys = networkTables.getStringArrayTopic(ROOT + "/PathKeys").publish();
        combinedPath = networkTables.getStructArrayTopic(ROOT + "/Path", Pose2d.struct).publish();
        eventPoses = networkTables.getStructArrayTopic(ROOT + "/Events", Pose2d.struct).publish();
        eventLabels = networkTables.getStringArrayTopic(ROOT + "/EventLabels").publish();
    }

    /** Publishes only when selection or declarative plan geometry changes. */
    public void publish(List<AutoPreview> previews) {
        AutoPreview preview = previews == null || previews.isEmpty()
                ? new AutoPreview("", List.of(), List.of()) : previews.get(0);
        String signature = signature(preview);
        if (lastSignature.equals(signature)) return;
        lastSignature = signature;

        selected.set(preview.name());
        plan.set(preview.steps().toArray(String[]::new));
        pathKeys.set(preview.paths().stream().map(PathPreview::key).toArray(String[]::new));

        List<Pose2d> combined = new ArrayList<>();
        List<Pose2d> events = new ArrayList<>();
        List<String> labels = new ArrayList<>();
        Map<String, Boolean> active = new LinkedHashMap<>();
        for (int index = 0; index < preview.paths().size(); index++) {
            PathPreview path = preview.paths().get(index);
            String channel = String.format("%02d_%s", index, clean(path.key()));
            Pose2d[] poses = path.poses().stream().map(AutoPreviewPublisher::pose).toArray(Pose2d[]::new);
            pathPublishers.computeIfAbsent(channel, key -> networkTables
                    .getStructArrayTopic(ROOT + "/Paths/" + key, Pose2d.struct).publish()).set(poses);
            active.put(channel, true);
            combined.addAll(List.of(poses));
            for (PathPreview.Event event : path.events()) {
                events.add(pose(event.pose()));
                labels.add(path.key() + " | " + event.name() + " | " + event.timeSeconds() + "s");
            }
        }
        pathPublishers.forEach((name, publisher) -> {
            if (!active.containsKey(name)) publisher.set(new Pose2d[0]);
        });
        combinedPath.set(combined.toArray(Pose2d[]::new));
        eventPoses.set(events.toArray(Pose2d[]::new));
        eventLabels.set(labels.toArray(String[]::new));
    }

    private static Pose2d pose(PathPreview.Pose pose) {
        return new Pose2d(pose.xMeters(), pose.yMeters(), Rotation2d.fromRadians(pose.headingRadians()));
    }

    private static String signature(AutoPreview preview) {
        int hash = Objects.hash(preview.name(), preview.steps());
        for (PathPreview path : preview.paths())
            hash = 31 * hash + Objects.hash(path.key(), path.poses(), path.events());
        return preview.name() + ":" + hash;
    }

    private static String clean(String value) {
        String safe = value == null ? "path" : value.replaceAll("[^A-Za-z0-9_.-]", "_");
        return safe.isBlank() ? "path" : safe;
    }

    @Override public void close() {
        selected.close();
        plan.close();
        pathKeys.close();
        combinedPath.close();
        eventPoses.close();
        eventLabels.close();
        pathPublishers.values().forEach(StructArrayPublisher::close);
        pathPublishers.clear();
    }
}
