package ca.frc6390.athena.arcp;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Runs without the native library: checks the JNI marshalling buffers and Config validation. */
class ArcpMarshalTest {

    @Test
    void valuesRoundTripThroughTheMarshallingBuffer() {
        ArcpMarshal.Out out = new ArcpMarshal.Out();
        List<Object> values = List.of(
                Boolean.TRUE,
                Long.MIN_VALUE,
                -42L,
                Double.NaN,
                1.25,
                "turret",
                new boolean[] {true, false},
                new long[] {7L, -9L},
                new double[] {0.5, -2.0},
                new String[] {"a", "bc"},
                new ArcpStruct(3L, new byte[] {1, 2, 3}));
        out.reset();
        for (Object v : values) {
            out.value(v);
        }
        ArcpMarshal.In in = new ArcpMarshal.In(out.buffer());
        assertEquals(Boolean.TRUE, in.value());
        assertEquals(Long.MIN_VALUE, in.value());
        assertEquals(-42L, in.value());
        assertTrue(Double.isNaN((Double) in.value()));
        assertEquals(1.25, in.value());
        assertEquals("turret", in.value());
        assertArrayEquals(new boolean[] {true, false}, (boolean[]) in.value());
        assertArrayEquals(new long[] {7L, -9L}, (long[]) in.value());
        assertArrayEquals(new double[] {0.5, -2.0}, (double[]) in.value());
        assertArrayEquals(new String[] {"a", "bc"}, (String[]) in.value());
        assertEquals(new ArcpStruct(3L, new byte[] {1, 2, 3}), in.value());
    }

    @Test
    void cycleBufferStartsWithCountAndSignalId() {
        ArcpMarshal.Out out = new ArcpMarshal.Out();
        out.cycle(new ArcpCycle().set(5L, 1.0));
        ArcpMarshal.In in = new ArcpMarshal.In(out.buffer());
        assertEquals(1, in.u32());
        assertEquals(5L, in.u64());
        assertEquals(ArcpMarshal.KIND_F64, in.u8());
        assertEquals(1.0, Double.longBitsToDouble(in.u64()));
    }

    @Test
    void unsupportedValueTypesAreRejected() {
        ArcpMarshal.Out out = new ArcpMarshal.Out();
        assertThrows(IllegalArgumentException.class, () -> out.value(Float.valueOf(1f)));
    }

    @Test
    void configRejectsOutOfRangeValues() {
        assertThrows(IllegalArgumentException.class,
                () -> new NativeArcpServer.Config(70_000, null, 0L, 1200, 1_000_000L));
        assertThrows(IllegalArgumentException.class,
                () -> new NativeArcpServer.Config(0, null, 0L, 64, 1_000_000L));
        assertThrows(IllegalArgumentException.class,
                () -> new NativeArcpServer.Config(0, Path.of("x"), -1L, 1200, 1_000_000L));
        assertEquals(0, NativeArcpServer.Config.defaults().controlPort());
    }

    @Test
    void stoppedSinkReportsNoSessions() {
        assertEquals(ArcpStatus.STOPPED, ArcpSink.NONE.status());
        ArcpSink.NONE.serviceRequests();
    }
}
