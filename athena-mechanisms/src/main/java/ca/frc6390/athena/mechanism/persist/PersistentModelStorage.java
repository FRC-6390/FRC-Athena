package ca.frc6390.athena.mechanism.persist;

import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.model.ModelHistory;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import ca.frc6390.athena.mechanism.model.ModelStorage;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

/**
 * {@link ModelStorage} on top of a {@link PersistentStore}. A characterization uses its field path as the key
 * (mechanism tree path plus field path, the same path telemetry uses), so each characterization owns a small set of
 * entries under that key:
 *
 * <ul>
 *   <li>{@code key/model}, {@code key/model.kinds}, {@code key/model.motion}: the active model</li>
 *   <li>{@code key/running}, ...: the continuous-learning estimate, restored as the seed of the running estimate</li>
 *   <li>{@code key/history}, {@code key/history.reasons}, {@code key/history.motion}: the snapshot history</li>
 * </ul>
 *
 * <p>Writes go to the store's memory and reach the file on the store's batched flush. The owned key set is
 * available from {@link #keys(String)} so the boot-time cleanup keeps these entries.</p>
 */
public final class PersistentModelStorage implements ModelStorage {
    private static final System.Logger LOGGER = System.getLogger(PersistentModelStorage.class.getName());
    private static final String MODEL = "/model";
    private static final String KINDS = "/model.kinds";
    private static final String MOTION = "/model.motion";
    private static final String RUNNING = "/running";
    private static final String RUNNING_KINDS = "/running.kinds";
    private static final String RUNNING_MOTION = "/running.motion";
    private static final String HISTORY = "/history";
    private static final String HISTORY_REASONS = "/history.reasons";
    private static final String HISTORY_MOTION = "/history.motion";

    private final PersistentStore store;

    public PersistentModelStorage(PersistentStore store) {
        this.store = Objects.requireNonNull(store, "store");
    }

    /** Returns every store key this storage may write for a characterization key. */
    public static Set<String> keys(String key) {
        Objects.requireNonNull(key, "key");
        return Set.of(key + MODEL, key + KINDS, key + MOTION, key + RUNNING, key + RUNNING_KINDS,
                key + RUNNING_MOTION, key + HISTORY, key + HISTORY_REASONS, key + HISTORY_MOTION);
    }

    @Override
    public void save(String key, MechanismModel model) {
        writeModel(key + MODEL, key + KINDS, key + MOTION, model);
    }

    @Override
    public Optional<MechanismModel> load(String key) {
        return readModel(key + MODEL, key + KINDS, key + MOTION);
    }

    @Override
    public void saveRunning(String key, MechanismModel model) {
        writeModel(key + RUNNING, key + RUNNING_KINDS, key + RUNNING_MOTION, model);
    }

    @Override
    public Optional<MechanismModel> loadRunning(String key) {
        return readModel(key + RUNNING, key + RUNNING_KINDS, key + RUNNING_MOTION);
    }

    @Override
    public void saveHistory(String key, List<ModelHistory.Snapshot> snapshots) {
        if (snapshots.isEmpty()) {
            store.remove(key + HISTORY);
            store.remove(key + HISTORY_REASONS);
            store.remove(key + HISTORY_MOTION);
            return;
        }
        List<Double> flat = new ArrayList<>();
        List<String> reasons = new ArrayList<>();
        MotionKind kind = snapshots.get(0).model().motionKind();
        for (ModelHistory.Snapshot snapshot : snapshots) {
            double[] vector = ModelCodec.encode(snapshot.model());
            flat.add(Double.isFinite(snapshot.timestampSeconds()) ? snapshot.timestampSeconds() : 0.0);
            flat.add((double) vector.length);
            for (double value : vector) {
                flat.add(value);
            }
            reasons.add(snapshot.reason().replace('\n', ' '));
        }
        store.put(key + HISTORY, flat.stream().mapToDouble(Double::doubleValue).toArray());
        store.put(key + HISTORY_REASONS, String.join("\n", reasons));
        store.put(key + HISTORY_MOTION, kind.name());
    }

    @Override
    public List<ModelHistory.Snapshot> loadHistory(String key) {
        Optional<Object> flat = store.get(key + HISTORY);
        Optional<Object> reasons = store.get(key + HISTORY_REASONS);
        Optional<Object> motion = store.get(key + HISTORY_MOTION);
        if (flat.isEmpty() && reasons.isEmpty() && motion.isEmpty()) {
            return List.of();
        }
        if (flat.isEmpty() || reasons.isEmpty() || motion.isEmpty()
                || !(flat.get() instanceof double[] values) || !(reasons.get() instanceof String text)) {
            rejected(key + HISTORY, "missing or invalid history metadata");
            return List.of();
        }
        MotionKind kind = parseKind(motion.get());
        if (kind == null) {
            rejected(key + HISTORY_MOTION, "unknown motion kind");
            return List.of();
        }
        String[] reasonList = text.split("\n", -1);
        List<ModelHistory.Snapshot> snapshots = new ArrayList<>();
        int at = 0;
        for (String reason : reasonList) {
            if (values.length - at < 2) {
                rejected(key + HISTORY, "truncated snapshot header");
                break;
            }
            double timestamp = values[at];
            double storedLength = values[at + 1];
            at += 2;
            if (storedLength < 0.0 || storedLength > values.length - at
                    || storedLength != Math.rint(storedLength)) {
                rejected(key + HISTORY, "invalid snapshot length");
                break;
            }
            int length = (int) storedLength;
            double[] vector = Arrays.copyOfRange(values, at, at + length);
            at += length;
            MechanismModel model = ModelCodec.decode(vector, kind, List.of());
            if (model != null) {
                snapshots.add(new ModelHistory.Snapshot(timestamp, reason, model));
            } else {
                rejected(key + HISTORY, "invalid snapshot model");
            }
        }
        return List.copyOf(snapshots);
    }

    private void writeModel(String vectorKey, String kindsKey, String motionKey, MechanismModel model) {
        Objects.requireNonNull(model, "model");
        store.put(vectorKey, ModelCodec.encode(model));
        store.put(kindsKey, String.join(",", model.motorKinds()));
        store.put(motionKey, model.motionKind().name());
    }

    private Optional<MechanismModel> readModel(String vectorKey, String kindsKey, String motionKey) {
        Optional<Object> vector = store.get(vectorKey);
        Optional<Object> motion = store.get(motionKey);
        Optional<Object> kinds = store.get(kindsKey);
        if (vector.isEmpty() && motion.isEmpty() && kinds.isEmpty()) {
            return Optional.empty();
        }
        if (vector.isEmpty() || motion.isEmpty() || !(vector.get() instanceof double[] values)) {
            rejected(vectorKey, "missing or invalid model metadata");
            return Optional.empty();
        }
        MotionKind kind = parseKind(motion.get());
        if (kind == null) {
            rejected(motionKey, "unknown motion kind");
            return Optional.empty();
        }
        if (kinds.isPresent() && !(kinds.get() instanceof String)) {
            rejected(kindsKey, "invalid motor kind metadata");
            return Optional.empty();
        }
        MechanismModel model = ModelCodec.decode(values, kind, parseKinds(kinds.map(String.class::cast).orElse("")));
        if (model == null) rejected(vectorKey, "invalid model vector");
        return Optional.ofNullable(model);
    }

    private static List<String> parseKinds(String text) {
        if (text.isEmpty()) {
            return List.of();
        }
        Set<String> kinds = new LinkedHashSet<>(Arrays.asList(text.split(",", -1)));
        return List.copyOf(kinds);
    }

    private static MotionKind parseKind(Object value) {
        if (!(value instanceof String name)) return null;
        try {
            return MotionKind.valueOf(name);
        } catch (IllegalArgumentException unknown) {
            return null;
        }
    }

    private static void rejected(String key, String reason) {
        LOGGER.log(System.Logger.Level.WARNING,
                "Ignoring persisted characterization data at {0}: {1}. Code defaults are kept for rejected models.",
                key, reason);
    }
}
