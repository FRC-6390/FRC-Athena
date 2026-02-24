package ca.frc6390.athena.core.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.TypedInputResolverExamples;
import org.junit.jupiter.api.Test;

final class TypedInputResolverExamplesTest {

    @Test
    void strictResolverReturnsRegisteredValues() {
        TypedInputResolver resolver = TypedInputResolverExamples.strictResolver();

        assertTrue(resolver.boolVal("enabled"));
        assertEquals(0.75, resolver.doubleVal("gain"), 1e-9);
        assertEquals(2, resolver.intVal("slot"));
        assertEquals("teleop", resolver.stringVal("mode"));
        assertNotNull(resolver.pose2dVal("target2d"));
        assertNotNull(resolver.pose3dVal("target3d"));
        assertEquals(
                "safe",
                resolver.objectVal("profile", TypedInputResolverExamples.ControlProfile.class).name());
    }

    @Test
    void strictResolverThrowsForMissingKey() {
        TypedInputResolver resolver = TypedInputResolverExamples.strictResolver();

        assertThrows(IllegalArgumentException.class, () -> resolver.boolVal("missing"));
        assertThrows(IllegalArgumentException.class, () -> resolver.stringVal("missing"));
    }

    @Test
    void lenientResolverReturnsFallbacks() {
        TypedInputResolver resolver = TypedInputResolverExamples.lenientResolver();

        assertFalse(resolver.boolVal("missing"));
        assertTrue(Double.isNaN(resolver.doubleVal("missing")));
        assertEquals(0, resolver.intVal("missing"));
        assertEquals("", resolver.stringVal("missing"));
    }

    @Test
    void mutableInputsOverrideConfiguredSuppliers() {
        TypedInputResolverExamples.ExampleMutableInputs mutables = new TypedInputResolverExamples.ExampleMutableInputs()
                .bool("enabled", false)
                .dbl("gain", 0.9)
                .intVal("slot", 7)
                .str("mode", "service");

        TypedInputResolver resolver = TypedInputResolverExamples.resolverWithMutableOverrides(mutables);

        assertFalse(resolver.boolVal("enabled"));
        assertEquals(0.9, resolver.doubleVal("gain"), 1e-9);
        assertEquals(7, resolver.intVal("slot"));
        assertEquals("service", resolver.stringVal("mode"));
    }

    @Test
    void objectTypeMismatchThrows() {
        TypedInputResolver resolver = TypedInputResolverExamples.strictResolver();
        assertThrows(IllegalArgumentException.class, () -> resolver.objectVal("profile", String.class));
    }
}
