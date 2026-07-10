package frc.robot.auto;

import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.Paths;
import edu.wpi.first.wpilibj.Timer;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Function;

/** A tiny provider showing how a non-vendor event source can drive Athena's PathGraph. */
public final class ExampleMarkerPathProvider implements PathProvider {
    private Function<String, Boolean> markerRunner = marker -> true;

    public void markers(Function<String, Boolean> markerRunner) {
        this.markerRunner = markerRunner;
    }

    @Override
    public PathAction path(String pathName) {
        return Paths.of("custom", pathName).seconds(2.0);
    }

    @Override
    public CommandAction load(String pathName) {
        Timer timer = new Timer();
        Set<String> activeMarkers = new LinkedHashSet<>();
        boolean[] intakeStarted = {false};
        boolean[] stowStarted = {false};

        return CommandAction.create("custom:" + pathName)
                .onInitialize(() -> {
                    timer.restart();
                    activeMarkers.clear();
                    intakeStarted[0] = false;
                    stowStarted[0] = false;
                })
                .onExecute(() -> {
                    if (!intakeStarted[0] && timer.hasElapsed(0.40)) {
                        activeMarkers.add("custom-intake");
                        intakeStarted[0] = true;
                    }
                    if (!stowStarted[0] && timer.hasElapsed(1.40)) {
                        activeMarkers.add("custom-stow");
                        stowStarted[0] = true;
                    }
                    for (Iterator<String> iterator = activeMarkers.iterator(); iterator.hasNext();) {
                        if (markerRunner.apply(iterator.next())) {
                            iterator.remove();
                        }
                    }
                })
                .until(() -> timer.hasElapsed(2.0) && activeMarkers.isEmpty())
                .onEnd(timer::stop)
                .build();
    }

    @Override
    public PathRuntime runtime() {
        return new PathRuntime() {
            @Override
            public boolean isFinished(PathAction path, MechanismContext context) {
                return context.timeInStateSeconds() >= 2.0;
            }
        };
    }
}
