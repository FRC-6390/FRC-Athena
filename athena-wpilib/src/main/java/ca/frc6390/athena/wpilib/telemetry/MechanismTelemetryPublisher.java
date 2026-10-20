package ca.frc6390.athena.wpilib.telemetry;

import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Meters;
import ca.frc6390.athena.mechanism.core.TelemetryAction;
import ca.frc6390.athena.mechanism.core.TelemetryNode;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.geometry.Circle2d;
import ca.frc6390.athena.runtime.geometry.Geometry2d;
import edu.wpi.first.math.geometry.Translation2d;
import ca.frc6390.athena.runtime.geometry.Polygon2d;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Publishes Athena's mechanism-scoped telemetry schema to NT4 dashboards.
 *
 * <p>Values are written only when they differ from the last value written, so an unchanged value costs no
 * NetworkTables call. Dashboard writes to writable values and action controls are detected through one listener
 * poller drained once per publish, instead of asking every entry for its last-change time every cycle (a JNI call
 * that takes the NetworkTables storage lock).</p>
 */
public final class MechanismTelemetryPublisher implements AutoCloseable {
    public static final String ROOT = "/Athena/Mechanisms";
    private static final double DEFAULT_CUSTOM_PERIOD_SECONDS = 0.10;
    private static final EnumSet<NetworkTableEvent.Kind> VALUE_EVENTS = EnumSet.of(NetworkTableEvent.Kind.kValueAll);

    private final NetworkTableInstance instance;
    /** Listener handle to the channel whose entry it watches. */
    private final Map<Integer, ChangeWatcher> watchers = new HashMap<>();
    private NetworkTableListenerPoller poller;
    private final List<ValueChannel> values = new ArrayList<>();
    /**
     * The channels of {@link #values} updated on every publish (writable values and points), in the same order. A
     * publish between two read-only periods walks only these instead of skipping the rest one by one.
     */
    private final List<ValueChannel> everyCycle = new ArrayList<>();
    /** One lock state per shared lock condition, read at most once per publish. */
    private final List<LockState> lockStates = new ArrayList<>();
    private final List<ValueChannel> allValues = new ArrayList<>();
    private final List<ActionChannel> actions = new ArrayList<>();
    private final Set<String> reportedFailures = new HashSet<>();
    private TelemetrySchema boundSchema;
    private double lastCustomPublish = Double.NEGATIVE_INFINITY;
    private double customPeriodSeconds = DEFAULT_CUSTOM_PERIOD_SECONDS;

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

    /** Sets the read-only diagnostic value period without delaying writable controls or actions. */
    public MechanismTelemetryPublisher readOnlyPeriod(Time period) {
        double periodSeconds = Objects.requireNonNull(period, "period").in(Seconds);
        if (!Double.isFinite(periodSeconds) || periodSeconds < 0.02 || periodSeconds > 5.0) {
            throw new IllegalArgumentException("Telemetry period must be between 0.02 and 5 seconds.");
        }
        customPeriodSeconds = periodSeconds;
        return this;
    }

    void publish(TelemetrySchema schema, double timestampSeconds) {
        Objects.requireNonNull(schema, "schema");
        if (schema != boundSchema) {
            bind(schema);
        }
        boolean publishReadOnly = timestampSeconds - lastCustomPublish >= customPeriodSeconds;
        if (publishReadOnly) {
            lastCustomPublish = timestampSeconds;
        }
        drainChangeEvents();
        for (int index = 0, size = lockStates.size(); index < size; index++) {
            lockStates.get(index).stale = true;
        }
        List<ValueChannel> due = publishReadOnly ? values : everyCycle;
        for (int index = 0, size = due.size(); index < size; index++) {
            ValueChannel channel = due.get(index);
            try {
                channel.update(publishReadOnly);
            } catch (RuntimeException exception) {
                reportOnce(channel.path, exception);
            }
        }
        for (int index = 0, size = actions.size(); index < size; index++) {
            actions.get(index).update();
        }
    }

    private LockState lockState(TelemetryValue value) {
        for (LockState state : lockStates) {
            if (state.source.sharesLockCondition(value)) return state;
        }
        LockState state = new LockState(value);
        lockStates.add(state);
        return state;
    }

    /**
     * Marks the channels whose entries changed since the last publish. NetworkTables queues a value event in the same
     * storage update that moves an entry's last-change time, so a channel without an event has no change to apply.
     */
    private void drainChangeEvents() {
        if (poller == null || watchers.isEmpty()) return;
        NetworkTableEvent[] events = poller.readQueue();
        for (NetworkTableEvent event : events) {
            ChangeWatcher watcher = watchers.get(event.listener);
            if (watcher != null) watcher.changePending = true;
        }
    }

    /** Watches an entry for value changes; {@link #changePending} is set by {@link #drainChangeEvents()}. */
    private ChangeWatcher watch(NetworkTableEntry entry) {
        if (poller == null) poller = new NetworkTableListenerPoller(instance);
        ChangeWatcher watcher = new ChangeWatcher(poller, poller.addListener(entry, VALUE_EVENTS));
        watchers.put(watcher.listener, watcher);
        return watcher;
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
            if (!value.isPublished()) return;
            String path = join(nodePath, name);
            ValueChannel channel = new ValueChannel(instance, path, value);
            allValues.add(channel);
            if (value.writable() && channel.entry != null) {
                channel.watcher = watch(channel.entry);
            }
            if (value.constant()) {
                try {
                    channel.update(true);
                } catch (RuntimeException exception) {
                    reportOnce(path, exception);
                }
            } else {
                values.add(channel);
                if (value.writable() || value.type() == TelemetryValue.Type.POINTS) {
                    everyCycle.add(channel);
                }
                if (channel.lockStatus != null) {
                    channel.lockState = lockState(value);
                }
            }
        });
        node.actions().forEach((name, action) -> {
            ActionChannel channel = new ActionChannel(instance, join(nodePath, name), action);
            channel.watcher = watch(channel.commandRunning);
            actions.add(channel);
        });
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
            result[0] = pose(rectangle.minimumX().in(Meters), rectangle.minimumY().in(Meters));
            result[1] = pose(rectangle.maximumX().in(Meters), rectangle.minimumY().in(Meters));
            result[2] = pose(rectangle.maximumX().in(Meters), rectangle.maximumY().in(Meters));
            result[3] = pose(rectangle.minimumX().in(Meters), rectangle.maximumY().in(Meters));
            result[4] = result[0];
            return result;
        }
        if (geometry instanceof Polygon2d polygon) {
            int count = polygon.vertices().size();
            Pose2d[] result = new Pose2d[count + 1];
            for (int index = 0; index < count; index++) {
                Translation2d point = polygon.vertices().get(index);
                result[index] = pose(point.getX(), point.getY());
            }
            result[count] = result[0];
            return result;
        }
        if (geometry instanceof Circle2d circle) {
            Pose2d[] result = new Pose2d[33];
            for (int index = 0; index < result.length; index++) {
                double angle = 2.0 * Math.PI * index / (result.length - 1);
                result[index] = pose(
                        circle.center().getX() + circle.radius().in(Meters) * Math.cos(angle),
                        circle.center().getY() + circle.radius().in(Meters) * Math.sin(angle));
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
        if (poller != null) {
            poller.close();
            poller = null;
        }
    }

    private void closeChannels() {
        for (ChangeWatcher watcher : watchers.values()) watcher.close();
        watchers.clear();
        for (ValueChannel channel : allValues) channel.close();
        for (ActionChannel channel : actions) channel.close();
        values.clear();
        everyCycle.clear();
        lockStates.clear();
        allValues.clear();
        actions.clear();
    }

    /**
     * The lock state of every value that reads the same lock condition. The condition is read once per publish, by
     * the first channel that needs it, instead of once per value.
     */
    private static final class LockState {
        private final TelemetryValue source;
        private boolean stale = true;
        private boolean locked;

        private LockState(TelemetryValue source) {
            this.source = source;
        }

        private boolean locked() {
            if (stale) {
                locked = source.locked();
                stale = false;
            }
            return locked;
        }
    }

    /** A value listener on one entry and whether it reported a change not yet handled. */
    private static final class ChangeWatcher {
        private final NetworkTableListenerPoller poller;
        private final int listener;
        private boolean changePending;

        private ChangeWatcher(NetworkTableListenerPoller poller, int listener) {
            this.poller = poller;
            this.listener = listener;
        }

        /** True when the entry may have changed: unwatched, or a value event is not handled yet. */
        private static boolean pending(ChangeWatcher watcher) {
            return watcher == null || watcher.changePending;
        }

        /** Marks the pending change handled. Left set when handling throws, so the next publish retries it. */
        private static void handled(ChangeWatcher watcher) {
            if (watcher != null) watcher.changePending = false;
        }

        private void close() {
            poller.removeListener(listener);
        }
    }

    private static final class ValueChannel implements AutoCloseable {
        private final String path;
        private final TelemetryValue value;
        private final NetworkTableEntry entry;
        private final StructArrayPublisher<Pose2d> geometryPublisher;
        private final StructArrayPublisher<Translation2d> pointsPublisher;
        private final NetworkTableEntry writeStatus;
        private final NetworkTableEntry lockStatus;
        /** Change listener on {@link #entry} for writable values; null otherwise. */
        private ChangeWatcher watcher;
        /** Shared lock state when the value has a lock group and is updated by publish; null otherwise. */
        private LockState lockState;
        private String lastWriteStatus = "ok";
        private boolean lastLocked;
        private boolean lockPublished;
        private long lastEntryChange;
        private double lastNumber;
        private boolean lastBoolean;
        private Object lastObject;
        private boolean published;

        private ValueChannel(NetworkTableInstance nt, String path, TelemetryValue value) {
            this.path = path;
            this.value = value;
            String topic = topic(path);
            writeStatus = value.writable() ? nt.getEntry(topic + "/WriteStatus") : null;
            lockStatus = value.lockGroup() != null ? nt.getEntry(topic + "/Lock") : null;
            if (value.type() == TelemetryValue.Type.GEOMETRY) {
                entry = null;
                geometryPublisher = nt.getStructArrayTopic(topic, Pose2d.struct).publish();
                pointsPublisher = null;
            } else if (value.type() == TelemetryValue.Type.POINTS) {
                entry = null;
                geometryPublisher = null;
                pointsPublisher = nt.getStructArrayTopic(topic, Translation2d.struct).publish();
            } else {
                entry = nt.getEntry(topic);
                geometryPublisher = null;
                pointsPublisher = null;
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
                    case NUMBER_ARRAY -> {
                        lastObject = ((double[]) value.value()).clone();
                        entry.setDefaultDoubleArray((double[]) lastObject);
                    }
                    case POINTS -> throw new IllegalStateException("Points use a struct-array publisher.");
                    case GEOMETRY -> throw new IllegalStateException("Geometry uses a struct-array publisher.");
                }
                published = true;
            } else if (value.writable()) {
                applyDashboardValue();
            }
            if (writeStatus != null) writeStatus.setString(lastWriteStatus);
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
            if (pointsPublisher != null) {
                Translation2d[] points = (Translation2d[]) value.value();
                if (!published || !(lastObject instanceof Translation2d[] previous)
                        || !Arrays.equals(previous, points)) {
                    Translation2d[] translated = new Translation2d[points.length];
                    for (int index = 0; index < points.length; index++) {
                        translated[index] = new Translation2d(points[index].getX(), points[index].getY());
                    }
                    pointsPublisher.set(translated);
                    // TelemetryValue.points returns a fresh copy on every read, so this array is not shared.
                    lastObject = points;
                    published = true;
                }
                return;
            }
            if (value.writable()) {
                // Only an entry with a value event can have a new last-change time; skip the JNI call otherwise.
                long changed = ChangeWatcher.pending(watcher) ? entry.getLastChange() : lastEntryChange;
                if (changed != lastEntryChange) {
                    applyDashboardValue();
                    lastEntryChange = entry.getLastChange();
                    rememberCurrentValue();
                    ChangeWatcher.handled(watcher);
                } else {
                    ChangeWatcher.handled(watcher);
                    if (publishLocal) publishWritableIfChanged();
                }
                publishLockStatus();
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
                case NUMBER_ARRAY -> {
                    double[] current = (double[]) value.value();
                    if (!published || !(lastObject instanceof double[] previous)
                            || !Arrays.equals(previous, current)) {
                        entry.setDoubleArray(current);
                        lastObject = current.clone();
                        published = true;
                    }
                }
                case POINTS -> throw new IllegalStateException("Points use a struct-array publisher.");
                case GEOMETRY -> throw new IllegalStateException("Geometry uses a struct-array publisher.");
            }
        }

        private void applyDashboardValue() {
            switch (value.type()) {
                case NUMBER -> {
                    double current = value.number();
                    double requested = entry.getDouble(current);
                    if (Double.compare(current, requested) != 0) dashboardWrite(requested);
                }
                case BOOLEAN -> {
                    boolean current = value.bool();
                    boolean requested = entry.getBoolean(current);
                    if (value.momentary()) {
                        if (requested) dashboardWrite(true);
                        if (requested) entry.setBoolean(false);
                    } else if (current != requested) {
                        dashboardWrite(requested);
                    }
                }
                case STRING -> {
                    String current = String.valueOf(value.value());
                    String requested = entry.getString(current);
                    if (!current.equals(requested)) dashboardWrite(requested);
                }
                case NUMBER_ARRAY -> throw new IllegalArgumentException("Numeric-array telemetry is read-only.");
                case POINTS -> throw new IllegalArgumentException("Point telemetry is read-only.");
                case GEOMETRY -> throw new IllegalArgumentException("Geometry telemetry is read-only.");
            }
        }

        /**
         * Applies a dashboard write through the value's lock gate. A rejected write leaves the value
         * unchanged, resets the published entry to the actual value, and reports the reason in WriteStatus.
         */
        private void dashboardWrite(Object requested) {
            TelemetryValue.WriteResult result = value.tryWrite(requested);
            String status = result.accepted() ? "ok" : result.reason();
            if (writeStatus != null && !status.equals(lastWriteStatus)) {
                writeStatus.setString(status);
                lastWriteStatus = status;
            }
            if (!result.accepted()) {
                resetEntry();
            }
        }

        private void resetEntry() {
            switch (value.type()) {
                case NUMBER -> entry.setDouble(value.number());
                case BOOLEAN -> entry.setBoolean(value.bool());
                case STRING -> entry.setString(String.valueOf(value.value()));
                default -> { }
            }
        }

        private void publishLockStatus() {
            if (lockStatus == null) return;
            boolean locked = lockState != null ? lockState.locked() : value.locked();
            if (!lockPublished || locked != lastLocked) {
                lockStatus.setString(value.lockGroup().getSimpleName() + (locked ? " (locked)" : " (unlocked)"));
                lastLocked = locked;
                lockPublished = true;
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
                case NUMBER_ARRAY -> throw new IllegalStateException("Numeric-array telemetry is read-only.");
                case POINTS -> throw new IllegalStateException("Point telemetry is read-only.");
                case GEOMETRY -> throw new IllegalStateException("Geometry telemetry is read-only.");
            }
        }

        private void rememberCurrentValue() {
            switch (value.type()) {
                case NUMBER -> lastNumber = value.number();
                case BOOLEAN -> lastBoolean = value.bool();
                case STRING -> lastObject = String.valueOf(value.value());
                case NUMBER_ARRAY -> lastObject = ((double[]) value.value()).clone();
                case POINTS -> lastObject = ((Translation2d[]) value.value()).clone();
                case GEOMETRY -> { }
            }
        }

        @Override
        public void close() {
            if (geometryPublisher != null) geometryPublisher.close();
            if (pointsPublisher != null) pointsPublisher.close();
            if (entry != null) entry.close();
            if (writeStatus != null) writeStatus.close();
            if (lockStatus != null) lockStatus.close();
        }
    }

    private static final class ActionChannel implements AutoCloseable {
        private final TelemetryAction action;
        private final ActionCommand command;
        private final CommandScheduler scheduler = CommandScheduler.getInstance();
        private final NetworkTableEntry[] constants;
        private final NetworkTableEntry commandRunning;
        /** Change listener on {@link #commandRunning}. */
        private ChangeWatcher watcher;
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
            long changed = ChangeWatcher.pending(watcher) ? commandRunning.getLastChange() : lastCommandChange;
            if (changed != lastCommandChange) {
                boolean requested = commandRunning.getBoolean(scheduler.isScheduled(command));
                if (requested && !scheduler.isScheduled(command)) scheduler.schedule(command);
                else if (!requested && scheduler.isScheduled(command)) scheduler.cancel(command);
                lastCommandChange = commandRunning.getLastChange();
            }
            ChangeWatcher.handled(watcher);
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
