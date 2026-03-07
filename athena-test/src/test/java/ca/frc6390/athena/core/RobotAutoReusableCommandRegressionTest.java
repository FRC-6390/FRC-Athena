package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RobotAutoReusableCommandRegressionTest {

    @AfterEach
    void cleanupScheduler() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        scheduler.run();
        scheduler.clearComposedCommands();
    }

    @Test
    void markerCommandOverloadCanBeDeferredTwice() {
        CapturingRegisterCtx ctx = new CapturingRegisterCtx();
        AtomicInteger executions = new AtomicInteger();
        Command marker = Commands.runOnce(executions::incrementAndGet).ignoringDisable(true);

        ctx.marker("fire", marker);
        Supplier<Command> supplier = ctx.markers.get("fire");
        assertNotNull(supplier);

        runToCompletion(Commands.defer(supplier, Set.of()).ignoringDisable(true));
        runToCompletion(Commands.defer(supplier, Set.of()).ignoringDisable(true));

        assertEquals(2, executions.get());
    }

    @Test
    void namedCommandCommandOverloadCanBeDeferredTwice() throws Exception {
        RobotAuto autos = new RobotAuto();
        AtomicInteger executions = new AtomicInteger();
        Command named = Commands.runOnce(executions::incrementAndGet).ignoringDisable(true);

        autos.registry().command("score", named);
        Supplier<Command> supplier = namedCommandSupplier(autos, "score");
        assertNotNull(supplier);

        runToCompletion(Commands.defer(supplier, Set.of()).ignoringDisable(true));
        runToCompletion(Commands.defer(supplier, Set.of()).ignoringDisable(true));

        assertEquals(2, executions.get());
    }

    @SuppressWarnings("unchecked")
    private static Supplier<Command> namedCommandSupplier(RobotAuto autos, String id) throws Exception {
        Field field = RobotAuto.class.getDeclaredField("namedCommandSuppliers");
        field.setAccessible(true);
        Map<String, Supplier<Command>> suppliers = (Map<String, Supplier<Command>>) field.get(autos);
        return suppliers.get(id);
    }

    private static void runToCompletion(Command command) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.schedule(command);
        for (int i = 0; i < 25 && scheduler.isScheduled(command); i++) {
            scheduler.run();
        }
        assertEquals(false, scheduler.isScheduled(command), "Command did not finish");
    }

    private static final class CapturingRegisterCtx implements RobotAuto.AutoRegisterCtx {
        private final Map<String, Supplier<Command>> markers = new LinkedHashMap<>();

        @Override
        public RobotCore<?> robot() {
            return null;
        }

        @Override
        public String programId() {
            return "test";
        }

        @Override
        public boolean hasMarker(String id) {
            return markers.containsKey(id);
        }

        @Override
        public boolean hasAuto(String id) {
            return false;
        }

        @Override
        public boolean hasInput(String id) {
            return false;
        }

        @Override
        public boolean hasInput(String programId, String id) {
            return false;
        }

        @Override
        public <V> Supplier<V> input(String programId, String key, Class<V> type) {
            return () -> null;
        }

        @Override
        public RobotAuto.AutoRegisterCtx marker(String id, Supplier<Command> supplier) {
            markers.put(id, supplier);
            return this;
        }

        @Override
        public <V> RobotAuto.AutoRegisterCtx registerInput(String key, Class<V> type, Supplier<V> supplier) {
            return this;
        }

        @Override
        public RobotAuto.AutoRegisterCtx path(String reference, RobotAuto.TrajectorySource source) {
            return this;
        }

        @Override
        public RobotAuto.AutoRegisterCtx path(String reference, RobotAuto.TrajectorySource source, String id) {
            return this;
        }

        @Override
        public RobotAuto.AutoRegisterCtx custom(String id, Supplier<Command> factory) {
            return this;
        }

        @Override
        public RobotAuto.AutoRegisterCtx auto(RobotAuto.AutoRoutine routine) {
            return this;
        }
    }
}
