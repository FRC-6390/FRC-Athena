package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.mechanism.core.TelemetryAction;
import ca.frc6390.athena.mechanism.core.TelemetryNode;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.geometry.Circle2d;
import ca.frc6390.athena.runtime.geometry.Geometry2d;
import ca.frc6390.athena.runtime.geometry.Point2d;
import ca.frc6390.athena.runtime.geometry.Polygon2d;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/** Publishes Athena's mechanism-scoped telemetry schema to NT4 dashboards. */
public final class MechanismTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/Mechanisms";
    private static final double CUSTOM_PERIOD_SECONDS = 0.10;

    private final NetworkTableInstance instance;
    private final List<ValueChannel> values = new ArrayList<>();
    private final List<ValueChannel> allValues = new ArrayList<>();
    private final List<ActionChannel> actions = new ArrayList<>();
    private final Set<String> reportedFailures = new HashSet<>();
    private TelemetrySchema boundSchema;
    private double lastCustomPublish = Double.NEGATIVE_INFINITY;

    public MechanismTelemetryPublisher() {
        this(NetworkTableInstance.getDefault());
    }

    MechanismTelemetryPublisher(NetworkTableInstance instance) {
        this.instance = Objects.requireNonNull(instance, "instance");
    }

    /** Publishes the latest values and services dashboard action controls. */
    public void publish(TelemetrySchema schema) {
        publish(schema, Timer.getFPGATimestamp());
    }

    void publish(TelemetrySchema schema, double timestampSeconds) {
        Objects.requireNonNull(schema, "schema");
        if (schema != boundSchema) {
            bind(schema);
        }
        boolean publishReadOnly = timestampSeconds - lastCustomPublish >= CUSTOM_PERIOD_SECONDS;
        if (publishReadOnly) {
            lastCustomPublish = timestampSeconds;
        }
        for (int index = 0, size = values.size(); index < size; index++) {
            ValueChannel channel = values.get(index);
            if (channel.value.writable() || publishReadOnly) {
                try {
                    channel.update(publishReadOnly);
                } catch (RuntimeException exception) {
                    reportOnce(channel.path, exception);
                }
            }
        }
        for (int index = 0, size = actions.size(); index < size; index++) {
            actions.get(index).update();
        }
    }

    private void bind(TelemetrySchema schema) {
        closeChannels();
        boundSchema = schema;
        bindNode(schema.root(), "");
        lastCustomPublish = Double.NEGATIVE_INFINITY;
    }

    private void bindNode(TelemetryNode node, String parentPath) {
        String nodePath = node.kind() == TelemetryNode.Kind.ROOT
                ? parentPath
                : join(parentPath, node.name());
        node.values().forEach((name, value) -> {
            String path = join(nodePath, name);
            ValueChannel channel = new ValueChannel(instance, path, value);
            allValues.add(channel);
            if (value.constant()) {
                try {
                    channel.update(true);
                } catch (RuntimeException exception) {
                    reportOnce(path, exception);
                }
            } else {
                values.add(channel);
            }
        });
        node.actions().forEach((name, action) ->
                actions.add(new ActionChannel(instance, join(nodePath, name), action)));
        node.children().forEach((name, child) -> bindNode(child, nodePath));
    }

    private void reportOnce(String path, RuntimeException exception) {
        if (reportedFailures.add(path)) {
            DriverStation.reportWarning(
                    "Athena telemetry '" + path + "' is unavailable: " + exception.getMessage(), false);
        }
    }

    static Pose2d[] outline(Geometry2d geometry) {
        if (geometry instanceof Rectangle2d rectangle) {
            Pose2d[] result = new Pose2d[5];
            result[0] = pose(rectangle.minimumX(), rectangle.minimumY());
            result[1] = pose(rectangle.maximumX(), rectangle.minimumY());
            result[2] = pose(rectangle.maximumX(), rectangle.maximumY());
            result[3] = pose(rectangle.minimumX(), rectangle.maximumY());
            result[4] = result[0];
            return result;
        }
        if (geometry instanceof Polygon2d polygon) {
            int count = polygon.vertices().size();
            Pose2d[] result = new Pose2d[count + 1];
            for (int index = 0; index < count; index++) {
                Point2d point = polygon.vertices().get(index);
                result[index] = pose(point.x(), point.y());
            }
            result[count] = result[0];
            return result;
        }
        if (geometry instanceof Circle2d circle) {
            Pose2d[] result = new Pose2d[33];
            for (int index = 0; index < result.length; index++) {
                double angle = 2.0 * Math.PI * index / (result.length - 1);
                result[index] = pose(
                        circle.center().x() + circle.radius() * Math.cos(angle),
                        circle.center().y() + circle.radius() * Math.sin(angle));
            }
            return result;
        }
        throw new IllegalArgumentException("Only Rectangle2d, Polygon2d, and Circle2d can be visualized.");
    }

    private static Pose2d pose(double x, double y) {
        return new Pose2d(x, y, Rotation2d.kZero);
    }

    private static String topic(String path) {
        String safe = sanitize(path);
        return ROOT + "/" + (safe.isEmpty() ? "value" : safe);
    }

    private static String sanitize(String path) {
        if (path == null || path.isBlank()) {
            return "";
        }
        int start = 0;
        while (start < path.length() && path.charAt(start) == '/') {
            start++;
        }
        StringBuilder cleaned = null;
        for (int index = start; index < path.length(); index++) {
            char character = path.charAt(index);
            boolean valid = Character.isLetterOrDigit(character)
                    || character == '_' || character == '.' || character == '/' || character == '-';
            if (!valid && cleaned == null) {
                cleaned = new StringBuilder(path.length()).append(path, start, index);
            }
            if (cleaned != null) {
                cleaned.append(valid ? character : '_');
            }
        }
        return cleaned == null ? path.substring(start) : cleaned.toString();
    }

    private static String join(String left, String right) {
        if (right == null || right.isBlank()) return left;
        return left == null || left.isBlank() ? right : left + "/" + right;
    }

    @Override
    public void close() {
        closeChannels();
        boundSchema = null;
    }

    private void closeChannels() {
        for (ValueChannel channel : allValues) channel.close();
        for (ActionChannel channel : actions) channel.close();
        values.clear();
        allValues.clear();
        actions.clear();
    }

    private static final class ValueChannel implements AutoCloseable {
        private final String path;
        private final TelemetryValue value;
        private final NetworkTableEntry entry;
        private final StructArrayPublisher<Pose2d> geometryPublisher;
        private long lastEntryChange;
        private double lastNumber;
        private boolean lastBoolean;
        private Object lastObject;
        private boolean published;

        private ValueChannel(NetworkTableInstance nt, String path, TelemetryValue value) {
            this.path = path;
            this.value = value;
            String topic = topic(path);
            if (value.type() == TelemetryValue.Type.GEOMETRY) {
                entry = null;
                geometryPublisher = nt.getStructArrayTopic(topic, Pose2d.struct).publish();
            } else {
                entry = nt.getEntry(topic);
                geometryPublisher = null;
                initializeEntry();
            }
        }

        private void initializeEntry() {
            boolean existed = entry.exists();
            if (!existed) {
                switch (value.type()) {
                    case NUMBER -> {
                        lastNumber = value.number();
                        entry.setDefaultDouble(lastNumber);
                    }
                    case BOOLEAN -> {
                        lastBoolean = value.bool();
                        entry.setDefaultBoolean(lastBoolean);
                    }
                    case STRING -> {
                        lastObject = String.valueOf(value.value());
                        entry.setDefaultString((String) lastObject);
                    }
                    case GEOMETRY -> throw new IllegalStateException("Geometry uses a struct-array publisher.");
                }
                published = true;
            } else if (value.writable()) {
                applyDashboardValue();
            }
            lastEntryChange = entry.getLastChange();
            rememberCurrentValue();
        }

        private void update(boolean publishLocal) {
            if (geometryPublisher != null) {
                Geometry2d geometry = (Geometry2d) value.value();
                if (!published || !Objects.equals(lastObject, geometry)) {
                    geometryPublisher.set(outline(geometry));
                    lastObject = geometry;
                    published = true;
                }
                return;
            }
            if (value.writable()) {
                long changed = entry.getLastChange();
                if (changed != lastEntryChange) {
                    applyDashboardValue();
                    lastEntryChange = entry.getLastChange();
                    rememberCurrentValue();
                } else if (publishLocal) {
                    publishWritableIfChanged();
                }
                return;
            }
            switch (value.type()) {
                case NUMBER -> {
                    double current = value.number();
                    if (!published || Double.compare(lastNumber, current) != 0) {
                        entry.setDouble(current);
                        lastNumber = current;
                        published = true;
                    }
                }
                case BOOLEAN -> {
                    boolean current = value.bool();
                    if (!published || lastBoolean != current) {
                        entry.setBoolean(current);
                        lastBoolean = current;
                        published = true;
                    }
                }
                case STRING -> {
                    String current = String.valueOf(value.value());
                    if (!published || !Objects.equals(lastObject, current)) {
                        entry.setString(current);
                        lastObject = current;
                        published = true;
                    }
                }
                case GEOMETRY -> throw new IllegalStateException("Geometry uses a struct-array publisher.");
            }
        }

        private void applyDashboardValue() {
            switch (value.type()) {
                case NUMBER -> {
                    double current = value.number();
                    double requested = entry.getDouble(current);
                    if (Double.compare(current, requested) != 0) value.set(requested);
                }
                case BOOLEAN -> {
                    boolean current = value.bool();
                    boolean requested = entry.getBoolean(current);
                    if (value.momentary()) {
                        if (requested) value.set(true);
                        if (requested) entry.setBoolean(false);
                    } else if (current != requested) {
                        value.set(requested);
                    }
                }
                case STRING -> {
                    String current = String.valueOf(value.value());
                    String requested = entry.getString(current);
                    if (!current.equals(requested)) value.set(requested);
                }
                case GEOMETRY -> throw new IllegalArgumentException("Geometry telemetry is read-only.");
            }
        }

        private void publishWritableIfChanged() {
            switch (value.type()) {
                case NUMBER -> {
                    double current = value.number();
                    if (Double.compare(lastNumber, current) != 0) {
                        entry.setDouble(current);
                        lastEntryChange = entry.getLastChange();
                        lastNumber = current;
                    }
                }
                case BOOLEAN -> {
                    boolean current = value.bool();
                    if (lastBoolean != current) {
                        entry.setBoolean(current);
                        lastEntryChange = entry.getLastChange();
                        lastBoolean = current;
                    }
                }
                case STRING -> {
                    String current = String.valueOf(value.value());
                    if (!Objects.equals(lastObject, current)) {
                        entry.setString(current);
                        lastEntryChange = entry.getLastChange();
                        lastObject = current;
                    }
                }
                case GEOMETRY -> throw new IllegalStateException("Geometry telemetry is read-only.");
            }
        }

        private void rememberCurrentValue() {
            switch (value.type()) {
                case NUMBER -> lastNumber = value.number();
                case BOOLEAN -> lastBoolean = value.bool();
                case STRING -> lastObject = String.valueOf(value.value());
                case GEOMETRY -> { }
            }
        }

        @Override
        public void close() {
            if (geometryPublisher != null) geometryPublisher.close();
            if (entry != null) entry.close();
        }
    }

    private static final class ActionChannel implements AutoCloseable {
        private final TelemetryAction action;
        private final ActionCommand command;
        private final CommandScheduler scheduler = CommandScheduler.getInstance();
        private final NetworkTableEntry[] constants;
        private final NetworkTableEntry commandRunning;
        private final NetworkTableEntry running;
        private final NetworkTableEntry complete;
        private long lastCommandChange;
        private boolean lastScheduled;
        private boolean lastRunning;
        private boolean lastComplete;
        private boolean publishedStatus;

        private ActionChannel(NetworkTableInstance nt, String path, TelemetryAction action) {
            this.action = action;
            command = new ActionCommand(action);
            String topic = topic(path);
            constants = new NetworkTableEntry[] {
                    stringEntry(nt, topic + "/.type", "Command"),
                    stringEntry(nt, topic + "/.name", action.name()),
                    stringEntry(nt, topic + "/interruptBehavior", command.getInterruptionBehavior().toString()),
                    booleanEntry(nt, topic + "/.isParented", false),
                    booleanEntry(nt, topic + "/runsWhenDisabled", command.runsWhenDisabled()),
                    stringEntry(nt, topic + "/ActionType", action.type())
            };
            commandRunning = nt.getEntry(topic + "/running");
            running = nt.getEntry(topic + "/Running");
            complete = nt.getEntry(topic + "/Complete");
            commandRunning.setDefaultBoolean(false);
            lastCommandChange = commandRunning.getLastChange();
        }

        private void update() {
            long changed = commandRunning.getLastChange();
            if (changed != lastCommandChange) {
                boolean requested = commandRunning.getBoolean(scheduler.isScheduled(command));
                if (requested && !scheduler.isScheduled(command)) scheduler.schedule(command);
                else if (!requested && scheduler.isScheduled(command)) scheduler.cancel(command);
                lastCommandChange = commandRunning.getLastChange();
            }
            boolean scheduled = scheduler.isScheduled(command);
            if (!publishedStatus || lastScheduled != scheduled) {
                commandRunning.setBoolean(scheduled);
                lastCommandChange = commandRunning.getLastChange();
                lastScheduled = scheduled;
            }
            boolean active = action.running();
            if (!publishedStatus || lastRunning != active) {
                running.setBoolean(active);
                lastRunning = active;
            }
            boolean done = action.complete();
            if (!publishedStatus || lastComplete != done) {
                complete.setBoolean(done);
                lastComplete = done;
            }
            publishedStatus = true;
        }

        private static NetworkTableEntry stringEntry(NetworkTableInstance nt, String path, String value) {
            NetworkTableEntry entry = nt.getEntry(path);
            entry.setString(value);
            return entry;
        }

        private static NetworkTableEntry booleanEntry(NetworkTableInstance nt, String path, boolean value) {
            NetworkTableEntry entry = nt.getEntry(path);
            entry.setBoolean(value);
            return entry;
        }

        @Override
        public void close() {
            if (scheduler.isScheduled(command)) scheduler.cancel(command);
            commandRunning.close();
            running.close();
            complete.close();
            for (NetworkTableEntry entry : constants) entry.close();
        }
    }

    /** A real WPILib command whose lifecycle delegates to one named Athena action. */
    static final class ActionCommand extends Command {
        private final TelemetryAction action;

        ActionCommand(TelemetryAction action) {
            this.action = action;
            setName(action.name());
        }

        @Override
        public void initialize() {
            action.request();
        }

        @Override
        public boolean isFinished() {
            return action.complete() && !action.running();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) action.cancel();
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}
