package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

final class RobotAutoInputOverridesTest {

    @Test
    void runtimeInputOverrideTakesPrecedenceUntilCleared() {
        RobotAuto autos = new RobotAuto();
        autos.inputs().string("handoff.mode", () -> "default");

        assertEquals("default", autos.inputs().string("handoff.mode"));

        autos.inputs().string("handoff.mode", "amp");
        assertEquals("amp", autos.inputs().string("handoff.mode"));

        autos.inputs().resetString("handoff.mode");
        assertEquals("default", autos.inputs().string("handoff.mode"));
    }

    @Test
    void clearAllRuntimeInputOverridesRestoresRegisteredSuppliers() {
        RobotAuto autos = new RobotAuto();
        autos.inputs().integer("handoff.count", () -> 0);
        autos.inputs().bool("handoff.fire", () -> false);

        autos.inputs().integer("handoff.count", 3);
        autos.inputs().bool("handoff.fire", true);
        assertEquals(3, autos.inputs().integer("handoff.count"));
        assertTrue(autos.inputs().bool("handoff.fire"));

        autos.inputs().clear();
        assertEquals(0, autos.inputs().integer("handoff.count"));
        assertEquals(false, autos.inputs().bool("handoff.fire"));
    }

    @Test
    void runtimeInputOverrideTypeMismatchFailsWhenRead() {
        RobotAuto autos = new RobotAuto();
        autos.inputs().integer("handoff.shared", () -> 1);
        autos.inputs().bool("handoff.shared", true);

        IllegalStateException ex = assertThrows(
                IllegalStateException.class,
                () -> autos.inputs().integer("handoff.shared"));
        assertTrue(ex.getMessage().contains("Runtime auto input type mismatch"));
    }

    @Test
    void scopedRuntimeOverridesDoNotCollideAcrossPrograms() throws Exception {
        RobotAuto autos = new RobotAuto();
        Map<String, Object> producerInputs = new LinkedHashMap<>();
        Map<String, Object> consumerInputs = new LinkedHashMap<>();

        invokeRegisterScopedInput(
                autos,
                producerInputs,
                "ProducerProgram",
                "handoff.mode",
                String.class,
                () -> "producer-default");
        invokeRegisterScopedInput(
                autos,
                consumerInputs,
                "ConsumerProgram",
                "handoff.mode",
                String.class,
                () -> "consumer-default");

        Supplier<String> producer = invokeInputSupplier(
                autos,
                producerInputs,
                "ProducerProgram",
                "handoff.mode",
                String.class);
        Supplier<String> consumer = invokeInputSupplier(
                autos,
                consumerInputs,
                "ConsumerProgram",
                "handoff.mode",
                String.class);

        assertEquals("producer-default", producer.get());
        assertEquals("consumer-default", consumer.get());

        invokeSetScopedOverride(autos, "ConsumerProgram", "handoff.mode", String.class, "amp");
        assertEquals("producer-default", producer.get());
        assertEquals("amp", consumer.get());
    }

    private static void invokeRegisterScopedInput(
            RobotAuto autos,
            Map<String, Object> scopedInputs,
            String programId,
            String key,
            Class<?> type,
            Supplier<?> supplier) throws Exception {
        Method method = RobotAuto.class.getDeclaredMethod(
                "registerScopedInput",
                Map.class,
                String.class,
                String.class,
                Class.class,
                Supplier.class);
        method.setAccessible(true);
        invoke(method, autos, scopedInputs, programId, key, type, supplier);
    }

    @SuppressWarnings("unchecked")
    private static <T> Supplier<T> invokeInputSupplier(
            RobotAuto autos,
            Map<String, Object> scopedInputs,
            String scope,
            String key,
            Class<T> type) throws Exception {
        Method method = RobotAuto.class.getDeclaredMethod(
                "inputSupplier",
                Map.class,
                String.class,
                String.class,
                Class.class);
        method.setAccessible(true);
        return invoke(method, autos, scopedInputs, scope, key, type);
    }

    private static void invokeSetScopedOverride(
            RobotAuto autos,
            String scope,
            String key,
            Class<?> type,
            Object value) throws Exception {
        Method method = RobotAuto.class.getDeclaredMethod(
                "setRuntimeInputOverride",
                String.class,
                String.class,
                Class.class,
                Object.class);
        method.setAccessible(true);
        invoke(method, autos, scope, key, type, value);
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
}
