package ca.frc6390.athena.auto;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Scoped autonomous input values shared while an auto execution is prepared.
 */
public final class AutoInputStore {
    private static final String GLOBAL_SCOPE = "global";
    private final Map<String, AutoInputScope> scopes = new LinkedHashMap<>();

    /**
     * Returns the global input scope.
     *
     * @return global scope
     */
    public AutoInputScope global() {
        return scope(GLOBAL_SCOPE);
    }

    /**
     * Returns a named routine input scope.
     *
     * @param name scope name
     * @return input scope
     */
    public AutoInputScope scope(String name) {
        return scopes.computeIfAbsent(normalizeScope(name), ignored -> new AutoInputScope());
    }

    /**
     * Clears one named scope.
     *
     * @param name scope name
     * @return this store
     */
    public AutoInputStore clearScope(String name) {
        scopes.remove(normalizeScope(name));
        return this;
    }

    /**
     * Clears all scopes.
     *
     * @return this store
     */
    public AutoInputStore clear() {
        scopes.clear();
        return this;
    }

    private static String normalizeScope(String name) {
        return name == null || name.isBlank() ? GLOBAL_SCOPE : name.trim();
    }
}
