package ca.frc6390.athena.commands;

/**
 * Minimal command runner for tests and non-WPILib execution.
 */
public final class CommandRunner {
    private final CommandSpec spec;
    private boolean initialized;
    private boolean ended;

    /**
     * Creates a runner.
     *
     * @param spec command spec
     */
    public CommandRunner(CommandSpec spec) {
        this.spec = spec;
    }

    /**
     * Runs one command cycle.
     *
     * @return true once the command has ended
     */
    public boolean step() {
        if (ended) {
            return true;
        }
        if (!initialized) {
            spec.onInitialize().run();
            initialized = true;
        }
        spec.onExecute().run();
        if (spec.isFinished().get()) {
            spec.onEnd().run();
            ended = true;
        }
        return ended;
    }
}
