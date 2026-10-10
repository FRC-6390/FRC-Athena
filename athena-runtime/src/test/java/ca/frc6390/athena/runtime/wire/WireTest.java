package ca.frc6390.athena.runtime.wire;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

class WireTest {
    @Test
    void primitivesRoundTrip() {
        WireWriter out = new WireWriter(4)
                .u8(0xAB).u16(0xBEEF).u32(0xDEADBEEFL).u64(-2L)
                .f64(Math.PI).varint(300).zigzag(-12345).string("turret/Ω");

        WireReader in = new WireReader(out.toByteArray());
        assertEquals(0xAB, in.u8());
        assertEquals(0xBEEF, in.u16());
        assertEquals(0xDEADBEEFL, in.u32());
        assertEquals(-2L, in.u64());
        assertEquals(Math.PI, in.f64());
        assertEquals(300, in.varint());
        assertEquals(-12345, in.zigzag());
        assertEquals("turret/Ω", in.string());
        assertFalse(in.hasRemaining());
    }

    @Test
    void varintMatchesLeb128() {
        assertArrayEquals(new byte[] {(byte) 0xAC, 0x02}, new WireWriter().varint(300).toByteArray());
        assertArrayEquals(new byte[] {0x03}, new WireWriter().zigzag(-2).toByteArray());
    }

    @Test
    void recordEnvelopeRoundTrips() {
        WireWriter payload = new WireWriter();
        Wire.marker(payload, 1_000_000L, Wire.MARKER_RECORD, "shot", 7);
        WireWriter out = new WireWriter().record(Wire.RECORD_MARKER, payload);

        WireReader.Record record = new WireReader(out.toByteArray()).record();
        assertEquals(Wire.RECORD_MARKER, record.type());
        assertEquals(1_000_000L, record.payload().zigzag());
        assertEquals(Wire.MARKER_RECORD, record.payload().varint());
        assertEquals("shot", record.payload().string());
        assertEquals(7, record.payload().varint());
    }

    @Test
    void frameHeaderLayout() {
        WireWriter out = new WireWriter();
        Wire.frameHeader(out, true, 20_000L, 5, 2);
        WireReader in = new WireReader(out.toByteArray());
        assertEquals(Wire.FRAME_VERSION, in.u8());
        assertEquals(Wire.FRAME_FLAG_KEYFRAME, in.u8());
        assertEquals(20_000L, in.u64());
        assertEquals(5, in.u32());
        assertEquals(0, in.u16());
        assertEquals(1, in.u16());
        assertEquals(2, in.varint());
    }
}
