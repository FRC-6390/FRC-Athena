package ca.frc6390.athena.runtime.wire;

/** Constants for the ARCP v2 / replay wire format (appendix E1-W of the off-season design). */
public final class Wire {
    public static final int FRAME_VERSION = 2;
    public static final int REPLAY_FORMAT_VERSION = 1;
    public static final byte[] REPLAY_MAGIC = {'A', 'T', 'H', 'R', 'P', 0};

    public static final int RECORD_DICTIONARY = 0x01;
    public static final int RECORD_FRAME = 0x02;
    public static final int RECORD_MARKER = 0x03;
    public static final int RECORD_METADATA = 0x04;
    public static final int RECORD_STRUCT_SCHEMA = 0x05;
    public static final int RECORD_COMPRESSED_BLOCK = 0x10;

    public static final int FRAME_FLAG_KEYFRAME = 0x01;
    public static final int FRAME_FLAG_FRAGMENT = 0x02;

    public static final int TYPE_BOOL = 1;
    public static final int TYPE_I64 = 2;
    public static final int TYPE_F64 = 3;
    public static final int TYPE_STRING = 4;
    public static final int TYPE_BOOL_ARRAY = 5;
    public static final int TYPE_I64_ARRAY = 6;
    public static final int TYPE_F64_ARRAY = 7;
    public static final int TYPE_STRING_ARRAY = 8;
    public static final int TYPE_STRUCT = 9;

    public static final int MARKER_BOOT = 1;
    public static final int MARKER_MATCH_IDENTIFIED = 2;
    public static final int MARKER_AUTO_START = 3;
    public static final int MARKER_TELEOP_START = 4;
    public static final int MARKER_MATCH_END = 5;
    public static final int MARKER_DISABLE = 6;
    public static final int MARKER_FAULT = 7;
    public static final int MARKER_STAGE_CHANGE = 8;
    public static final int MARKER_CHECK = 9;
    public static final int MARKER_OVERRIDE = 10;
    public static final int MARKER_PERSISTED_WRITE = 11;
    public static final int MARKER_RECORD = 12;
    public static final int MARKER_TEST_START = 13;
    public static final int MARKER_BROWNOUT = 14;
    public static final int MARKER_USER_BASE = 100;

    private Wire() {
    }

    /**
     * Writes a frame header. Entries follow as {@code varint signalId, u8 typeTag, value}.
     *
     * @param out destination
     * @param keyframe whether the frame carries every signal
     * @param cycleTimestampMicros FPGA cycle timestamp
     * @param sequence frame sequence number
     * @param entryCount number of entries that follow
     */
    public static void frameHeader(
            WireWriter out, boolean keyframe, long cycleTimestampMicros, long sequence, int entryCount) {
        out.u8(FRAME_VERSION)
                .u8(keyframe ? FRAME_FLAG_KEYFRAME : 0)
                .u64(cycleTimestampMicros)
                .u32(sequence)
                .u16(0)
                .u16(1)
                .varint(entryCount);
    }

    /** Writes a MARKER payload (wrap with {@link WireWriter#record}). */
    public static void marker(WireWriter out, long timestampMicros, int kind, String message, long signalId) {
        out.zigzag(timestampMicros).varint(kind).string(message == null ? "" : message).varint(signalId);
    }
}
