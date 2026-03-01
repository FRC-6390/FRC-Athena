package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RobotAutoScopedResolutionTest {

    @AfterEach
    void cleanupScheduler() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.run();
    }

    @Test
    void deferredScopedAutoFallsBackToGlobalRoutine() throws Exception {
        RobotAuto autos = new RobotAuto();
        List<String> events = new ArrayList<>();
        autos.registry().routine("climb", () -> Commands.runOnce(() -> events.add("climb")).ignoringDisable(true));

        Command command = invokeDeferredScopedAuto(
                autos,
                new LinkedHashMap<>(),
                new LinkedHashMap<>(),
                "climb",
                OptionalInt.empty());

        runToCompletion(command);
        assertEquals(List.of("climb"), events);
    }

    @Test
    void deferredScopedAutoFallsBackToGlobalExplicitSplit() throws Exception {
        RobotAuto autos = new RobotAuto();
        List<String> events = new ArrayList<>();
        autos.registry().routine(
                "climb.2",
                () -> Commands.runOnce(() -> events.add("climb-split-2")).ignoringDisable(true));

        Command command = invokeDeferredScopedAuto(
                autos,
                new LinkedHashMap<>(),
                new LinkedHashMap<>(),
                "climb",
                OptionalInt.of(2));

        runToCompletion(command);
        assertEquals(List.of("climb-split-2"), events);
    }

    @Test
    void scopedOdometryResetCanUseGlobalRoutinePose() throws Exception {
        RobotAuto autos = new RobotAuto();
        List<String> events = new ArrayList<>();
        RobotAuto.AutoRoutine climbRoutine = new RobotAuto.AutoRoutine(
                RobotAuto.AutoKey.of("climb"),
                RobotAuto.AutoSource.CUSTOM,
                "climb",
                () -> Commands.runOnce(() -> events.add("climb")).ignoringDisable(true),
                new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90.0)),
                true,
                null);
        autos.registry().routine(climbRoutine);

        Map<String, RobotAuto.AutoRoutine> scopedAutos = new LinkedHashMap<>();
        Map<String, Supplier<Command>> scopedMarkers = new LinkedHashMap<>();
        Command base = invokeDeferredScopedAuto(autos, scopedAutos, scopedMarkers, "climb", OptionalInt.empty());
        Command wrapped = invokeApplyScopedBuildCtxOdometryReset(
                autos,
                scopedAutos,
                "climb",
                OptionalInt.empty(),
                base,
                RobotAuto.AutoBuildCtx.OdometryResetTarget.PATH_START,
                null);

        runToCompletion(wrapped);
        assertEquals(List.of("climb"), events);
    }

    private static Command invokeDeferredScopedAuto(
            RobotAuto autos,
            Map<String, RobotAuto.AutoRoutine> scopedAutos,
            Map<String, Supplier<Command>> scopedMarkers,
            String id,
            OptionalInt splitIndex) throws Exception {
        Method method = RobotAuto.class.getDeclaredMethod(
                "deferredScopedAuto",
                Map.class,
                Map.class,
                String.class,
                OptionalInt.class);
        method.setAccessible(true);
        return invoke(method, autos, scopedAutos, scopedMarkers, id, splitIndex);
    }

    private static Command invokeApplyScopedBuildCtxOdometryReset(
            RobotAuto autos,
            Map<String, RobotAuto.AutoRoutine> scopedAutos,
            String id,
            OptionalInt splitIndex,
            Command command,
            RobotAuto.AutoBuildCtx.OdometryResetTarget target,
            Pose2d explicitPose) throws Exception {
        Method method = RobotAuto.class.getDeclaredMethod(
                "applyScopedBuildCtxOdometryReset",
                Map.class,
                String.class,
                OptionalInt.class,
                Command.class,
                RobotAuto.AutoBuildCtx.OdometryResetTarget.class,
                Pose2d.class);
        method.setAccessible(true);
        return invoke(method, autos, scopedAutos, id, splitIndex, command, target, explicitPose);
    }

    @SuppressWarnings("unchecked")
    private static <T> T invoke(Method method, Object target, Object... args) throws Exception {
        try {
            return (T) method.invoke(target, args);
        } catch (InvocationTargetException ex) {
            Throwable cause = ex.getCause();
            if (cause instanceof Exception checked) {
                throw checked;
            }
            throw new RuntimeException(cause);
        }
    }

    private static void runToCompletion(Command command) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Command runnable = command.ignoringDisable(true);
        scheduler.schedule(runnable);
        for (int i = 0; i < 25 && scheduler.isScheduled(runnable); i++) {
            scheduler.run();
        }
        assertFalse(scheduler.isScheduled(runnable), "Command did not finish");
    }
}
