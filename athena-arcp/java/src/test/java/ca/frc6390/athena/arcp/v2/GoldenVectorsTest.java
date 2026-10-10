package ca.frc6390.athena.arcp.v2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireReader;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.zip.Deflater;
import org.junit.jupiter.api.Test;

/**
 * Generates and checks the shared golden wire vectors in {@code docs/wire-vectors}.
 *
 * <p>The vectors are produced with the reference codec in {@code athena-runtime}
 * ({@link WireWriter}, {@link Wire}), not with the ARCP codec in this module, so they are an
 * independent oracle. Run with {@code ATHENA_REGEN_VECTORS=1} to rewrite the files.</p>
 */
final class GoldenVectorsTest {

    private static final long CYCLE_TIMESTAMP = 1_234_567_890L;
    /** Fully qualified lock group; writable signals carry it in lockGroup. */
    static final String LOCK_GROUP = "ca.frc6390.athena.runtime.Locks.InMatch";

    static Path vectorDir() {
        String root = System.getProperty("athena.repoRoot");
        if (root == null) {
            root = System.getenv().getOrDefault("ATHENA_REPO_ROOT", ".");
        }
        return Path.of(root, "docs", "wire-vectors");
    }

    /** Name to bytes for every vector. Insertion order is the documentation order. */
    static Map<String, byte[]> vectors() {
        Map<String, byte[]> out = new LinkedHashMap<>();
        byte[] dictionary = dictionary();
        byte[] structSchema = structSchema();
        byte[] metadata = metadata();
        byte[] keyframe = keyframeAllTypes();
        byte[] delta = deltaFrame();
        byte[] fragment = fragmentFrame();
        byte[] markers = markers();
        out.put("dictionary.hex", dictionary);
        out.put("struct_schema.hex", structSchema);
        out.put("metadata.hex", metadata);
        out.put("frame_keyframe_all_types.hex", keyframe);
        out.put("frame_delta.hex", delta);
        out.put("frame_fragment.hex", fragment);
        out.put("marker.hex", markers);
        out.put("compressed_block.hex", compressedBlock(concat(keyframe, delta, markers)));
        out.put("replay_sample.athrp.hex", replaySample(metadata, dictionary, structSchema, keyframe, delta, fragment, markers));
        return out;
    }

    @Test
    void goldenVectorsMatchReferenceCodec() throws IOException {
        Path dir = vectorDir();
        boolean regen = System.getenv("ATHENA_REGEN_VECTORS") != null;
        for (Map.Entry<String, byte[]> entry : vectors().entrySet()) {
            String expected = hex(entry.getValue()) + "\n";
            Path file = dir.resolve(entry.getKey());
            if (regen) {
                Files.createDirectories(dir);
                Files.writeString(file, expected, StandardCharsets.US_ASCII);
            }
            assertTrue(Files.exists(file), "missing golden vector " + file);
            assertEquals(expected, Files.readString(file, StandardCharsets.US_ASCII), entry.getKey());
        }
    }

    @Test
    void referenceReaderWalksEveryVector() throws IOException {
        for (Map.Entry<String, byte[]> entry : vectors().entrySet()) {
            if (entry.getKey().endsWith(".athrp.hex")) {
                continue;
            }
            WireReader reader = new WireReader(entry.getValue());
            int records = 0;
            while (reader.hasRemaining()) {
                reader.record();
                records++;
            }
            assertTrue(records > 0, entry.getKey());
        }
    }

    // ---- record builders -------------------------------------------------------------------

    static byte[] dictionary() {
        WireWriter payload = new WireWriter();
        payload.varint(5);
        dictEntry(payload, 1, "robot/turret/angle", 3, "deg", 1, 1 | 2, LOCK_GROUP, 0);
        dictEntry(payload, 2, "robot/turret/enabled", 1, "", 2, 1, "", 0);
        dictEntry(payload, 3, "robot/drive/pose", 9, "m", 4, 0, "", 1);
        dictEntry(payload, 4, "robot/mode", 4, "", 1, 0, "", 0);
        // Lock state: boolean role-7 signal at athena/locks/<fully.qualified.GroupClassName>.
        dictEntry(payload, 5, "athena/locks/" + LOCK_GROUP, 1, "", 7, 0, "", 0);
        return record(Wire.RECORD_DICTIONARY, payload);
    }

    private static void dictEntry(WireWriter w, long id, String path, int tag, String unit, int role,
            int flags, String lockGroup, long schemaId) {
        w.varint(id).string(path).u8(tag).string(unit).u8(role).u8(flags).string(lockGroup).varint(schemaId);
    }

    static byte[] structSchema() {
        WireWriter payload = new WireWriter();
        payload.varint(1).string("Pose2d").string("Translation2d translation;Rotation2d rotation");
        return record(Wire.RECORD_STRUCT_SCHEMA, payload);
    }

    static byte[] metadata() {
        WireWriter payload = new WireWriter();
        payload.varint(5);
        payload.string("event").string("Practice");
        payload.string("matchType").string("PRACTICE");
        payload.string("matchNumber").string("3");
        payload.string("robot").string("Athena");
        payload.string("athenaVersion").string("2027.0.0-SNAPSHOT");
        return record(Wire.RECORD_METADATA, payload);
    }

    /** One keyframe carrying every type tag 1-9. Signal ids are tag * 37 to exercise multi-byte varints. */
    static byte[] keyframeAllTypes() {
        WireWriter payload = new WireWriter();
        Wire.frameHeader(payload, true, CYCLE_TIMESTAMP, 42, 9);
        // tag 1 bool
        payload.varint(1 * 37).u8(1).u8(1);
        // tag 2 i64
        payload.varint(2 * 37).u8(2).varint(zz(-1234567890123L));
        // tag 3 f64
        payload.varint(3 * 37).u8(3).f64(3.25);
        // tag 4 string
        payload.varint(4 * 37).u8(4).string("hello ✓");
        // tag 5 bool[]
        payload.varint(5 * 37).u8(5).varint(3).u8(1).u8(0).u8(1);
        // tag 6 i64[]
        payload.varint(6 * 37).u8(6).varint(3).varint(zz(0)).varint(zz(-1)).varint(zz(Long.MAX_VALUE));
        // tag 7 f64[]
        payload.varint(7 * 37).u8(7).varint(2).f64(1.5).f64(-2.25);
        // tag 8 string[]
        payload.varint(8 * 37).u8(8).varint(3).string("a").string("").string("bc");
        // tag 9 struct: Pose2d (3 doubles = 24 bytes)
        payload.varint(9 * 37).u8(9).varint(1).varint(24);
        payload.f64(1.0).f64(2.0).f64(0.5);
        return record(Wire.RECORD_FRAME, payload);
    }

    static byte[] deltaFrame() {
        WireWriter payload = new WireWriter();
        Wire.frameHeader(payload, false, CYCLE_TIMESTAMP + 20_000, 43, 2);
        payload.varint(2 * 37).u8(2).varint(zz(-1234567889999L));
        payload.varint(37).u8(1).u8(0);
        return record(Wire.RECORD_FRAME, payload);
    }

    static byte[] fragmentFrame() {
        WireWriter payload = new WireWriter();
        // Fragment 1 of 2: flags bit1 set, fragment index 1, fragment count 2.
        payload.u8(Wire.FRAME_VERSION).u8(Wire.FRAME_FLAG_FRAGMENT).u64(CYCLE_TIMESTAMP + 40_000).u32(44)
                .u16(1).u16(2).varint(1);
        payload.varint(4 * 37).u8(4).string("fragmented");
        return record(Wire.RECORD_FRAME, payload);
    }

    static byte[] markers() {
        WireWriter first = new WireWriter();
        Wire.marker(first, -5000L, Wire.MARKER_FAULT, "motor fault", 0);
        WireWriter second = new WireWriter();
        Wire.marker(second, CYCLE_TIMESTAMP + 60_000, Wire.MARKER_USER_BASE, "user", 3 * 37);
        return concat(record(Wire.RECORD_MARKER, first), record(Wire.RECORD_MARKER, second));
    }

    /** COMPRESSED_BLOCK: varint uncompressedLength, then raw DEFLATE of the inner record stream. */
    static byte[] compressedBlock(byte[] inner) {
        byte[] deflated = deflateRaw(inner);
        WireWriter payload = new WireWriter();
        payload.varint(inner.length).bytes(deflated, 0, deflated.length);
        return record(Wire.RECORD_COMPRESSED_BLOCK, payload);
    }

    /** Replay file: ATHRP\0, u16 version, plain header records, then two compressed blocks. */
    static byte[] replaySample(byte[] metadata, byte[] dictionary, byte[] structSchema, byte[] keyframe,
            byte[] delta, byte[] fragment, byte[] markers) {
        WireWriter header = new WireWriter();
        for (byte b : Wire.REPLAY_MAGIC) {
            header.u8(b);
        }
        header.u16(Wire.REPLAY_FORMAT_VERSION);
        byte[] magic = header.toByteArray();
        return concat(magic, metadata, dictionary, structSchema,
                compressedBlock(concat(keyframe, delta, markers)),
                compressedBlock(concat(keyframe, fragment)));
    }

    // ---- helpers -----------------------------------------------------------------------------

    static byte[] record(int type, WireWriter payload) {
        WireWriter out = new WireWriter();
        out.record(type, payload);
        return out.toByteArray();
    }

    static byte[] concat(byte[]... parts) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        for (byte[] part : parts) {
            out.writeBytes(part);
        }
        return out.toByteArray();
    }

    static long zz(long value) {
        return (value << 1) ^ (value >> 63);
    }

    static byte[] deflateRaw(byte[] input) {
        Deflater deflater = new Deflater(Deflater.BEST_COMPRESSION, true);
        deflater.setInput(input);
        deflater.finish();
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buffer = new byte[1024];
        while (!deflater.finished()) {
            int n = deflater.deflate(buffer);
            out.write(buffer, 0, n);
        }
        deflater.end();
        return out.toByteArray();
    }

    static String hex(byte[] bytes) {
        StringBuilder sb = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) {
            sb.append(Character.forDigit((b >> 4) & 0xF, 16)).append(Character.forDigit(b & 0xF, 16));
        }
        return sb.toString();
    }
}
