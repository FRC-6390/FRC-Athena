package frc.robot.auto;

import ca.frc6390.athena.auto.PathPreview;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

/** Small vendor-neutral provider for generated straight-line paths. */
public final class GeneratedPathProvider implements PathProvider {
    public static final String KEY = "generated";
    private final ExampleDrive drive;

    public GeneratedPathProvider(ExampleDrive drive) {
        this.drive = drive;
    }

    @Override
    public String source() {
        return KEY;
    }

    @Override
    public PathRuntime runtime() {
        return new PathRuntime() {
            private final Map<String, Boolean> pendingReset = new ConcurrentHashMap<>();

            @Override
            public void initialize(PathAction path, MechanismContext context) {
                if (path.resetsOdometry()) {
                    pendingReset.put(path.key(), true);
                }
            }

            @Override
            public Action output(PathAction path, MechanismContext context) {
                drive.autoVelocity.set(RobotVelocity.robot(1.0, 0.0, 0.0));
                return drive.pooledDrive;
            }

            @Override
            public Map<String, Action> activeMarkers(PathAction path, MechanismContext context) {
                return pendingReset.remove(path.key()) == null
                        ? Map.of()
                        : Map.of("@resetOdometry", drive.resetPose(new Pose2d()));
            }

            @Override
            public boolean isFinished(PathAction path, MechanismContext context) {
                return context.timeInStateSeconds() >= path.expectedDurationSeconds().orElse(2.0);
            }

            @Override
            public void end(PathAction path, MechanismContext context, boolean interrupted) {
                pendingReset.remove(path.key());
                drive.autoVelocity.clear();
            }
        };
    }

    @Override
    public Optional<PathPreview> preview(PathAction path) {
        if (!owns(path)) {
            return Optional.empty();
        }
        return Optional.of(new PathPreview(path.key(), List.of(
                new PathPreview.Pose(0.0, 0.0, 0.0),
                new PathPreview.Pose(1.0, 0.0, 0.0),
                new PathPreview.Pose(2.0, 0.0, 0.0))));
    }
}
