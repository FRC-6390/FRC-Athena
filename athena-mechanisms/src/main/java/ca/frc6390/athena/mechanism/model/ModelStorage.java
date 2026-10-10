package ca.frc6390.athena.mechanism.model;

import java.util.List;
import java.util.Optional;

/**
 * Destination for identified models. The default is {@link InMemoryModelStorage}. The persistent store supplies
 * {@code PersistentModelStorage} without changing the characterization API.
 *
 * <p>Besides the active model, storage keeps the continuous-learning estimate and the snapshot history of one
 * characterization, under the same key. The defaults discard them, so an implementation only needs
 * {@link #save} and {@link #load} to keep the model.</p>
 */
public interface ModelStorage {
    /** Saves the model under a stable key. Replaces any previous model with the same key. */
    void save(String key, MechanismModel model);

    /** Loads the last model saved under a key. */
    Optional<MechanismModel> load(String key);

    /** Saves the continuous-learning estimate under a key. */
    default void saveRunning(String key, MechanismModel model) {
    }

    /** Loads the continuous-learning estimate saved under a key. */
    default Optional<MechanismModel> loadRunning(String key) {
        return Optional.empty();
    }

    /** Replaces the snapshot history under a key, oldest first. An empty list clears it. */
    default void saveHistory(String key, List<ModelHistory.Snapshot> snapshots) {
    }

    /** Loads the snapshot history under a key, oldest first. */
    default List<ModelHistory.Snapshot> loadHistory(String key) {
        return List.of();
    }
}
