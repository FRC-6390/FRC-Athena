package ca.frc6390.athena.robot.explore;

import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

/**
 * Result of {@link SimExplorer#run()}. Equal seeds, robot code and options give equal reports.
 *
 * @param seed run seed
 * @param durationSeconds requested run length
 * @param cycles robot cycles executed
 * @param findings one entry per category and subject, ordered by first occurrence
 */
public record ExplorationReport(long seed, double durationSeconds, long cycles, List<Finding> findings) {
    public ExplorationReport {
        findings = List.copyOf(findings);
    }

    /** True when the run produced no findings at all. */
    public boolean clean() {
        return findings.isEmpty();
    }

    /** Returns the findings of one category. */
    public List<Finding> findings(Category category) {
        return findings.stream().filter(finding -> finding.category() == category).toList();
    }

    /** Multi-line text summary, including shrunk input sequences and saved replay paths. */
    public String describe() {
        StringBuilder text = new StringBuilder(String.format(Locale.ROOT,
                "seed=%d duration=%.1fs cycles=%d findings=%d%n", seed, durationSeconds, cycles, findings.size()));
        for (Finding finding : findings) {
            text.append(String.format(Locale.ROOT, "- [%s] t=%.3fs %s (x%d)%n", finding.category(),
                    finding.timeSeconds(), finding.message(), finding.occurrences()));
            if (!finding.minimalInputs().isEmpty() || finding.shrunk()) {
                text.append(String.format(Locale.ROOT, "  minimal input (%d events, %.3fs):%n",
                        finding.minimalInputs().size(), finding.minimalDurationSeconds()));
                for (InputEvent event : finding.minimalInputs()) {
                    text.append("    ").append(event.describe()).append('\n');
                }
            }
            if (finding.replay() != null) {
                text.append("  replay: ").append(finding.replay()).append('\n');
            }
        }
        return text.toString();
    }

    /** Problem categories. {@link Category#STAGE_ENTRY} is informational. */
    public enum Category {
        /** A requested root Action lost every output to a newer lease for longer than the starvation limit. */
        STARVATION,
        /** An {@code atMost} or {@code Actions.timeout} bound expired before its Action completed. */
        COMMITMENT_TIMEOUT,
        /** A path finished with markers that never triggered. */
        AUTO_MARKER_PENDING,
        /** An effort ceiling clamped a command ({@link ca.frc6390.athena.mechanism.core.RuntimeEvents#EFFORT_CEILING}). */
        EFFORT_CEILING,
        /** A control's requested target was limited by its declared constraints. */
        CONSTRAINT_VIOLATION,
        /** A control's output was held back by a guard. */
        GUARD_BLOCK,
        /** Runtime fault contained by fault isolation, or raised by {@code Actions.fault}. */
        FAULT,
        /** A staging moved to a new stage. Informational. */
        STAGE_ENTRY,
        /** An exception escaped the robot runtime. The run stops at that cycle. */
        EXCEPTION;

        /** True for categories that describe normal behaviour rather than a problem. */
        public boolean informational() {
            return this == STAGE_ENTRY;
        }
    }

    /**
     * One finding.
     *
     * @param category finding category
     * @param subject mechanism, control, action or message that the finding is about
     * @param timeSeconds first occurrence
     * @param occurrences number of times the finding was observed in the run
     * @param message human-readable detail
     * @param history input events and mode changes before the first occurrence, oldest first (at most 40)
     * @param shrunk true when the input sequence was minimized
     * @param minimalInputs minimized input sequence that reproduces the category (empty when not shrunk)
     * @param minimalDurationSeconds run length of the minimized reproduction
     * @param replay saved {@code .athrp} file of the minimized run, or null
     */
    public record Finding(
            Category category,
            String subject,
            double timeSeconds,
            int occurrences,
            String message,
            List<String> history,
            boolean shrunk,
            List<InputEvent> minimalInputs,
            double minimalDurationSeconds,
            Path replay) {
        public Finding {
            Objects.requireNonNull(category, "category");
            subject = subject == null ? "" : subject;
            message = message == null ? "" : message;
            history = List.copyOf(history);
            minimalInputs = List.copyOf(minimalInputs);
        }
    }
}
