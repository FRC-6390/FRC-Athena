package ca.frc6390.athena.mechanism.model;

import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/** Process-local {@link ModelStorage}. Values are lost when the robot code restarts. */
public final class InMemoryModelStorage implements ModelStorage {
    private final Map<String, MechanismModel> models = new ConcurrentHashMap<>();

    @Override
    public void save(String key, MechanismModel model) {
        models.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(model, "model"));
    }

    @Override
    public Optional<MechanismModel> load(String key) {
        return Optional.ofNullable(models.get(key));
    }
}
