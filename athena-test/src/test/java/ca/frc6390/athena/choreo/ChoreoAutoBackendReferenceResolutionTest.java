package ca.frc6390.athena.choreo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.OptionalInt;
import org.junit.jupiter.api.Test;

final class ChoreoAutoBackendReferenceResolutionTest {

    @Test
    void parseTreatsNegativeSuffixAsLiteralReference() throws Exception {
        Object ref = parse("example.-1");
        assertEquals("example.-1", name(ref));
        assertTrue(split(ref).isEmpty());
    }

    @Test
    void parseTreatsNonNumericSuffixAsLiteralReference() throws Exception {
        Object ref = parse("example.alpha");
        assertEquals("example.alpha", name(ref));
        assertTrue(split(ref).isEmpty());
    }

    @Test
    void parseTreatsNumericSuffixAsSplitReference() throws Exception {
        Object ref = parse("example.2");
        assertEquals("example", name(ref));
        OptionalInt split = split(ref);
        assertTrue(split.isPresent());
        assertEquals(2, split.getAsInt());
    }

    private static Object parse(String reference) throws Exception {
        Class<?> refClass = Class.forName("ca.frc6390.athena.choreo.ChoreoAutoBackend$TrajectoryRef");
        Method parse = refClass.getDeclaredMethod("parse", String.class);
        parse.setAccessible(true);
        return parse.invoke(null, reference);
    }

    private static String name(Object ref) throws Exception {
        Method name = ref.getClass().getDeclaredMethod("name");
        name.setAccessible(true);
        return (String) name.invoke(ref);
    }

    private static OptionalInt split(Object ref) throws Exception {
        Method split = ref.getClass().getDeclaredMethod("splitIndex");
        split.setAccessible(true);
        return (OptionalInt) split.invoke(ref);
    }
}
