package frc.robot.auto;

import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.Paths;

public final class TimedPathProvider implements PathProvider {
    private final String source;
    private final double seconds;

    public TimedPathProvider(String source, double seconds) {
        this.source = source;
        this.seconds = seconds;
    }

    @Override
    public PathAction path(String pathName) {
        return Paths.of(source, pathName).seconds(seconds);
    }

    @Override
    public CommandAction load(String pathName) {
        return CommandAction.create(source + ":" + pathName)
                .onExecute(() -> {})
                .until(() -> false)
                .build();
    }

    @Override
    public PathRuntime runtime() {
        return new PathRuntime() {
            @Override
            public boolean isFinished(PathAction path, MechanismContext context) {
                return context.timeInStateSeconds() >= path.expectedDurationSeconds().orElse(seconds);
            }
        };
    }
}
