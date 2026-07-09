package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.lang.reflect.Modifier;
import org.junit.jupiter.api.Test;

class PublicMechanismApiSurfaceTest {
    @Test
    void oldMechanismRobotRuntimeClassIsRemoved() {
        assertThrows(
                ClassNotFoundException.class,
                () -> Class.forName("ca.frc6390.athena.mechanism.core.RobotRuntime"));
    }

    @Test
    void mechanismRuntimeInternalsAreNotPublicApi() {
        assertNotPublic(MechanismRuntime.class);
        assertNotPublic(MechanismIntrospector.class);
        assertNotPublic(HookIntrospector.class);
        assertNotPublic(PathIntrospector.class);
        assertNotPublic(OutputResolver.class);
        assertNotPublic(OutputApplier.class);
    }

    private static void assertNotPublic(Class<?> type) {
        assertFalse(Modifier.isPublic(type.getModifiers()), type.getName() + " must stay internal");
    }
}
