package ca.frc6390.athena.core.sections;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Shared helper for sectioned lambda access patterns.
 */
public final class SectionedAccess {
    private SectionedAccess() {}

    public static <O, S> O apply(O owner, Consumer<S> section, Supplier<S> sectionFactory) {
        if (section != null) {
            section.accept(sectionFactory.get());
        }
        return owner;
    }
}
