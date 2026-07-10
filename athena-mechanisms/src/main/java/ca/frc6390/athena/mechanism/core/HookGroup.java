package ca.frc6390.athena.mechanism.core;

import java.util.Map;

/**
 * A declaration that owns hook bindings discovered with its containing mechanism.
 */
public interface HookGroup {
    Map<String, HookBinding> hooks();
}
