package ca.frc6390.athena.pathplanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.lang.reflect.Method;
import java.util.List;
import org.junit.jupiter.api.Test;

final class PathPlannerAutoBackendPoseResolutionTest {
    private static final double EPSILON = 1e-9;

    @Test
    void resolvePathPosesInjectsStartAndEndHeading() throws Exception {
        Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(20.0));
        Pose2d mid = new Pose2d(2.3, 1.6, Rotation2d.fromDegrees(45.0));
        Pose2d end = new Pose2d(4.2, 2.4, Rotation2d.fromDegrees(100.0));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, mid, end);
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(3.0, 3.0, Math.PI, Math.PI),
                new IdealStartingState(0.0, start.getRotation()),
                new GoalEndState(0.0, end.getRotation()));

        List<Pose2d> raw = path.getPathPoses();
        assertFalse(raw.isEmpty());
        assertEquals(0.0, raw.get(0).getRotation().getDegrees(), EPSILON);
        assertEquals(0.0, raw.get(raw.size() - 1).getRotation().getDegrees(), EPSILON);

        List<Pose2d> resolved = resolvePathPoses(path, false);
        assertEquals(raw.size(), resolved.size());
        assertPoseEquals(start.getRotation(), resolved.get(0).getRotation());
        assertPoseEquals(end.getRotation(), resolved.get(resolved.size() - 1).getRotation());
    }

    @Test
    void resolvePathPosesFlipMatchesFlippedPathCoordinates() throws Exception {
        Pose2d start = new Pose2d(0.9, 0.8, Rotation2d.fromDegrees(-30.0));
        Pose2d mid = new Pose2d(2.1, 1.7, Rotation2d.fromDegrees(10.0));
        Pose2d end = new Pose2d(3.6, 2.5, Rotation2d.fromDegrees(75.0));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, mid, end);
        PathPlannerPath base = new PathPlannerPath(
                waypoints,
                new PathConstraints(2.5, 2.5, Math.PI, Math.PI),
                new IdealStartingState(0.0, start.getRotation()),
                new GoalEndState(0.0, end.getRotation()));
        PathPlannerPath flipped = base.flipPath();

        List<Pose2d> resolvedFlipFlag = resolvePathPoses(base, true);
        List<Pose2d> resolvedFromFlipped = resolvePathPoses(flipped, false);

        assertEquals(resolvedFromFlipped.size(), resolvedFlipFlag.size());
        for (int i = 0; i < resolvedFlipFlag.size(); i++) {
            Pose2d expected = resolvedFromFlipped.get(i);
            Pose2d actual = resolvedFlipFlag.get(i);
            assertEquals(expected.getX(), actual.getX(), EPSILON);
            assertEquals(expected.getY(), actual.getY(), EPSILON);
            assertPoseEquals(expected.getRotation(), actual.getRotation());
        }
    }

    @SuppressWarnings("unchecked")
    private static List<Pose2d> resolvePathPoses(PathPlannerPath path, boolean flipForAlliance) throws Exception {
        Method method = PathPlannerAutoBackend.class.getDeclaredMethod(
                "resolvePathPoses",
                PathPlannerPath.class,
                boolean.class);
        method.setAccessible(true);
        Object result = method.invoke(null, path, flipForAlliance);
        assertNotNull(result);
        return (List<Pose2d>) result;
    }

    private static void assertPoseEquals(Rotation2d expected, Rotation2d actual) {
        assertEquals(expected.getDegrees(), actual.getDegrees(), EPSILON);
    }
}
