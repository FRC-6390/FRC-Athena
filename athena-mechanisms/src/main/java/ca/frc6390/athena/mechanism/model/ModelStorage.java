package ca.frc6390.athena.mechanism.model;

import java.util.Optional;

/**
 * Destination for identified models. The default is {@link InMemoryModelStorage}. The persistent
 * store can supply its own implementation without changing the characterization API.
 */
public interface ModelStorage {
    /** Saves the model under a stable key. Replaces any previous model with the same key. */
    void save(String key, MechanismModel model);

    /** Loads the last model saved under a key. */
    Optional<MechanismModel> load(String key);
}
