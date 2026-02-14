package ca.frc6390.athena.drivetrains;

import java.util.function.Consumer;
import java.util.function.Supplier;

import ca.frc6390.athena.core.sections.SectionedAccess;

/**
 * Shared section-entry helper for drivetrain config surfaces.
 */
public abstract class SectionedDrivetrainConfig<Self> {
    protected abstract Self self();

    protected final <S> Self applySection(Consumer<S> section, Supplier<S> sectionFactory) {
        return SectionedAccess.apply(self(), section, sectionFactory);
    }
}
