package ca.frc6390.athena.localization.config;

import java.util.function.Consumer;

import ca.frc6390.athena.localization.spec.LocalizationSpec;

/**
 * Factory methods for localization declarations.
 */
public final class Localizations {
    private Localizations() {
    }

    /**
     * Creates and lowers a named localization declaration.
     *
     * @param name localization name
     * @param customizer declaration customizer
     * @return immutable localization spec
     */
    public static LocalizationSpec localization(String name, Consumer<LocalizationConfig> customizer) {
        LocalizationConfig config = LocalizationConfig.create(name);
        if (customizer != null) {
            customizer.accept(config);
        }
        return config.toSpec();
    }
}
