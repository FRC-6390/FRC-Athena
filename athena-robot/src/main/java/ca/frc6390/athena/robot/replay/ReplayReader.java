package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireReader;
import java.io.Closeable;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

/**
 * Reads {@code .athrp} files. The reader indexes compressed blocks on open; a {@link Cursor} decodes events
 * sequentially and can jump to any block. Every block starts with a dictionary and a keyframe, so seeking to
 * a block needs no earlier data.
 */
public final class ReplayReader implements Closeable {
    private record BlockRef(long payloadOffset, int payloadLength) {
    }

    private static final long UNKNOWN_START = Long.MIN_VALUE;

    private final RandomAccessFile file;
    private final Map<String, String> metadata;
    private final List<BlockRef> blocks;
    private final long[] blockStarts;

    private ReplayReader(RandomAccessFile file, Map<String, String> metadata, List<BlockRef> blocks) {
        this.file = file;
        this.metadata = Collections.unmodifiableMap(metadata);
        this.blocks = List.copyOf(blocks);
        this.blockStarts = new long[this.blocks.size()];
        Arrays.fill(blockStarts, UNKNOWN_START);
    }

    /**
     * Opens and indexes a replay file.
     *
     * @param path replay file
     * @return reader
     * @throws IOException when the file cannot be read or is not a replay file
     */
    public static ReplayReader open(Path path) throws IOException {
        RandomAccessFile file = new RandomAccessFile(path.toFile(), "r");
        try {
            byte[] magic = new byte[Wire.REPLAY_MAGIC.length];
            file.readFully(magic);
            if (!Arrays.equals(magic, Wire.REPLAY_MAGIC)) {
                throw new IOException("Not an Athena replay file: " + path);
            }
            int version = file.readUnsignedByte() | (file.readUnsignedByte() << 8);
            if (version != Wire.REPLAY_FORMAT_VERSION) {
                throw new IOException("Unsupported replay format version " + version + ".");
            }
            Map<String, String> metadata = new LinkedHashMap<>();
            List<BlockRef> blocks = new ArrayList<>();
            long position = file.getFilePointer();
            long length = file.length();
            while (position < length) {
                file.seek(position);
                int type = file.readUnsignedByte();
                long payloadLength = readVarint(file);
                long payloadOffset = file.getFilePointer();
                if (payloadOffset + payloadLength > length) {
                    throw new IOException("Truncated replay record at byte " + position + ".");
                }
                if (type == Wire.RECORD_COMPRESSED_BLOCK) {
                    blocks.add(new BlockRef(payloadOffset, (int) payloadLength));
                } else if (type == Wire.RECORD_METADATA) {
                    byte[] payload = new byte[(int) payloadLength];
                    file.seek(payloadOffset);
                    file.readFully(payload);
                    WireReader reader = new WireReader(payload);
                    long count = reader.varint();
                    for (long index = 0; index < count; index++) {
                        metadata.put(reader.string(), reader.string());
                    }
                }
                position = payloadOffset + payloadLength;
            }
            return new ReplayReader(file, metadata, blocks);
        } catch (IOException | RuntimeException exception) {
            file.close();
            throw exception;
        }
    }

    /** Returns METADATA key/value pairs. */
    public Map<String, String> metadata() {
        return metadata;
    }

    /** Returns the number of compressed blocks. */
    public int blockCount() {
        return blocks.size();
    }

    /** Returns a new cursor positioned before the first event. */
    public Cursor cursor() {
        return new Cursor(this);
    }

    /**
     * Returns the timestamp of the first frame or marker in a block. The block is decoded on the first call and
     * the result is cached, so a time seek costs one decode per probed block.
     *
     * @param index block index
     * @return first timestamp in microseconds, or {@link Long#MIN_VALUE} when the block holds no events
     * @throws IOException when the block cannot be read
     */
    public long blockStartMicros(int index) throws IOException {
        if (index < 0 || index >= blocks.size()) {
            throw new IndexOutOfBoundsException("Block " + index + " is out of range.");
        }
        if (blockStarts[index] == UNKNOWN_START) {
            long start = Long.MIN_VALUE;
            WireReader records = new WireReader(readBlock(index));
            while (records.hasRemaining()) {
                WireReader.Record record = records.record();
                if (record.type() == Wire.RECORD_FRAME) {
                    WireReader payload = record.payload();
                    payload.u8();
                    payload.u8();
                    start = payload.u64();
                    break;
                }
                if (record.type() == Wire.RECORD_MARKER) {
                    start = record.payload().zigzag();
                    break;
                }
            }
            blockStarts[index] = start;
        }
        return blockStarts[index];
    }

    private byte[] readBlock(int index) throws IOException {
        BlockRef ref = blocks.get(index);
        byte[] payload = new byte[ref.payloadLength()];
        file.seek(ref.payloadOffset());
        file.readFully(payload);
        WireReader reader = new WireReader(payload);
        int uncompressedLength = (int) reader.varint();
        int headerLength = payload.length - reader.remaining();
        Inflater inflater = new Inflater(true);
        try {
            inflater.setInput(payload, headerLength, payload.length - headerLength);
            byte[] output = new byte[uncompressedLength];
            int produced = 0;
            while (produced < uncompressedLength) {
                int count = inflater.inflate(output, produced, uncompressedLength - produced);
                if (count == 0 && (inflater.finished() || inflater.needsInput())) {
                    break;
                }
                produced += count;
            }
            if (produced != uncompressedLength) {
                throw new IOException("Corrupt replay block " + index + ".");
            }
            return output;
        } catch (DataFormatException exception) {
            throw new IOException("Corrupt replay block " + index + ".", exception);
        } finally {
            inflater.end();
        }
    }

    private static long readVarint(RandomAccessFile file) throws IOException {
        long value = 0;
        int shift = 0;
        while (true) {
            int next = file.readUnsignedByte();
            value |= (long) (next & 0x7F) << shift;
            if ((next & 0x80) == 0) {
                return value;
            }
            shift += 7;
            if (shift > 63) {
                throw new IOException("Invalid varint in replay file.");
            }
        }
    }

    @Override
    public void close() throws IOException {
        file.close();
    }

    /**
     * Sequential decoder over frames and markers. Dictionary records are consumed internally and update
     * {@link #signals()}. Frame values persist between calls, so {@link #f64(String)} and friends always return
     * the latest value of each signal.
     */
    public static final class Cursor {
        private final ReplayReader reader;
        private final Map<Integer, ReplaySignal> signalsById = new HashMap<>();
        private final Map<String, Integer> idsByPath = new HashMap<>();
        private long[] bits = new long[256];
        private Object[] objects = new Object[256];
        private boolean[] present = new boolean[256];
        private int nextBlock;
        private WireReader records;
        private int dictionaryVersion;
        private boolean frame;
        private boolean keyframe;
        private long sequence;
        private long timestampMicros;
        private int markerKind;
        private String markerMessage = "";
        private int markerSignalId;
        private boolean current;

        private Cursor(ReplayReader reader) {
            this.reader = reader;
        }

        /** Seeks so the next call to {@link #next()} starts at a block. Block starts are keyframes. */
        public void seekBlock(int index) {
            if (index < 0 || index >= reader.blockCount()) {
                throw new IndexOutOfBoundsException("Block " + index + " is out of range.");
            }
            nextBlock = index;
            records = null;
            current = false;
        }

        /**
         * Positions the cursor at the first frame or marker whose timestamp is at or after {@code micros}. The block
         * containing that point is found by binary search over block start times, and decoding starts at its
         * keyframe, so no earlier block is read. The next call to {@link #next()} returns that event.
         *
         * @param micros target time in microseconds (the same clock as {@link #timestampMicros()})
         * @throws IOException when a block cannot be read
         */
        public void seekTime(long micros) throws IOException {
            current = false;
            if (reader.blockCount() == 0) {
                return;
            }
            int low = 0;
            int high = reader.blockCount() - 1;
            int chosen = 0;
            while (low <= high) {
                int middle = (low + high) >>> 1;
                if (reader.blockStartMicros(middle) <= micros) {
                    chosen = middle;
                    low = middle + 1;
                } else {
                    high = middle - 1;
                }
            }
            seekBlock(chosen);
            while (next()) {
                if (timestampMicros >= micros) {
                    current = true;
                    return;
                }
            }
        }

        /**
         * Advances to the next frame or marker. After {@link #seekTime(long)} the first call returns the event the seek
         * stopped on.
         *
         * @return false at the end of the file
         * @throws IOException when a block cannot be read
         */
        public boolean next() throws IOException {
            if (current) {
                current = false;
                return true;
            }
            while (true) {
                if (records == null || !records.hasRemaining()) {
                    if (nextBlock >= reader.blockCount()) {
                        return false;
                    }
                    records = new WireReader(reader.readBlock(nextBlock++));
                    continue;
                }
                WireReader.Record record = records.record();
                switch (record.type()) {
                    case Wire.RECORD_DICTIONARY -> readDictionary(record.payload());
                    case Wire.RECORD_FRAME -> {
                        readFrame(record.payload());
                        return true;
                    }
                    case Wire.RECORD_MARKER -> {
                        readMarker(record.payload());
                        return true;
                    }
                    default -> {
                        // Unknown or informational records are skipped.
                    }
                }
            }
        }

        /** True when the current event is a frame. */
        public boolean isFrame() {
            return frame;
        }

        public boolean keyframe() {
            return keyframe;
        }

        public long sequence() {
            return sequence;
        }

        public long timestampMicros() {
            return timestampMicros;
        }

        public int markerKind() {
            return markerKind;
        }

        public String markerMessage() {
            return markerMessage;
        }

        public int markerSignalId() {
            return markerSignalId;
        }

        /** Increases whenever the dictionary gains or changes an entry. */
        public int dictionaryVersion() {
            return dictionaryVersion;
        }

        /** Returns the dictionary in id order. */
        public List<ReplaySignal> signals() {
            List<ReplaySignal> ordered = new ArrayList<>(signalsById.values());
            ordered.sort((a, b) -> Integer.compare(a.id(), b.id()));
            return List.copyOf(ordered);
        }

        public boolean has(String path) {
            Integer id = idsByPath.get(path);
            return id != null && present[id];
        }

        public double f64(String path) {
            Integer id = idsByPath.get(path);
            return id == null || !present[id] ? Double.NaN : Double.longBitsToDouble(bits[id]);
        }

        public boolean bool(String path) {
            Integer id = idsByPath.get(path);
            return id != null && present[id] && bits[id] != 0;
        }

        public long i64(String path) {
            Integer id = idsByPath.get(path);
            return id == null || !present[id] ? 0L : bits[id];
        }

        public String string(String path) {
            Integer id = idsByPath.get(path);
            if (id == null || !present[id]) return "";
            return objects[id] instanceof String value ? value : "";
        }

        /** Returns the decoded value of an array or struct signal, or null. */
        public Object value(String path) {
            Integer id = idsByPath.get(path);
            return id == null || !present[id] ? null : objects[id];
        }

        private void readDictionary(WireReader payload) {
            long count = payload.varint();
            for (long index = 0; index < count; index++) {
                ReplaySignal signal = new ReplaySignal(
                        (int) payload.varint(),
                        payload.string(),
                        payload.u8(),
                        payload.string(),
                        payload.u8(),
                        payload.u8(),
                        payload.string(),
                        payload.varint());
                ReplaySignal previous = signalsById.put(signal.id(), signal);
                if (previous != null) {
                    idsByPath.remove(previous.path());
                }
                idsByPath.put(signal.path(), signal.id());
                ensureCapacity(signal.id() + 1);
                if (!signal.equals(previous)) {
                    dictionaryVersion++;
                }
            }
        }

        private void readFrame(WireReader payload) {
            int version = payload.u8();
            if (version != Wire.FRAME_VERSION) {
                throw new IllegalStateException("Unsupported frame version " + version + ".");
            }
            int flags = payload.u8();
            timestampMicros = payload.u64();
            sequence = payload.u32();
            payload.u16();
            payload.u16();
            long entries = payload.varint();
            keyframe = (flags & Wire.FRAME_FLAG_KEYFRAME) != 0;
            for (long index = 0; index < entries; index++) {
                int id = (int) payload.varint();
                int type = payload.u8();
                ReplaySignal signal = signalsById.get(id);
                if (signal == null) {
                    throw new IllegalStateException("Frame references unknown signal " + id + ".");
                }
                ensureCapacity(id + 1);
                switch (type) {
                    case Wire.TYPE_BOOL -> bits[id] = payload.u8() != 0 ? 1L : 0L;
                    case Wire.TYPE_I64 -> bits[id] = payload.zigzag();
                    case Wire.TYPE_F64 -> bits[id] = payload.u64();
                    case Wire.TYPE_STRING -> objects[id] = payload.string();
                    case Wire.TYPE_BOOL_ARRAY -> {
                        int length = (int) payload.varint();
                        boolean[] values = new boolean[length];
                        for (int i = 0; i < length; i++) values[i] = payload.u8() != 0;
                        objects[id] = values;
                    }
                    case Wire.TYPE_I64_ARRAY -> {
                        int length = (int) payload.varint();
                        long[] values = new long[length];
                        for (int i = 0; i < length; i++) values[i] = payload.zigzag();
                        objects[id] = values;
                    }
                    case Wire.TYPE_F64_ARRAY -> {
                        int length = (int) payload.varint();
                        double[] values = new double[length];
                        for (int i = 0; i < length; i++) values[i] = payload.f64();
                        objects[id] = values;
                    }
                    case Wire.TYPE_STRING_ARRAY -> {
                        int length = (int) payload.varint();
                        String[] values = new String[length];
                        for (int i = 0; i < length; i++) values[i] = payload.string();
                        objects[id] = values;
                    }
                    case Wire.TYPE_STRUCT -> {
                        long schema = payload.varint();
                        int length = (int) payload.varint();
                        objects[id] = new StructValue(schema, payload.bytes(length));
                    }
                    default -> throw new IllegalStateException("Unknown type tag " + type + ".");
                }
                present[id] = true;
            }
            frame = true;
        }

        private void readMarker(WireReader payload) {
            timestampMicros = payload.zigzag();
            markerKind = (int) payload.varint();
            markerMessage = payload.string();
            markerSignalId = (int) payload.varint();
            frame = false;
        }

        private void ensureCapacity(int required) {
            if (required <= present.length) return;
            int size = Math.max(required, present.length * 2);
            bits = Arrays.copyOf(bits, size);
            objects = Arrays.copyOf(objects, size);
            present = Arrays.copyOf(present, size);
        }
    }

    /**
     * A struct-typed value: its schema id and raw WPILib-layout bytes.
     *
     * @param schemaId struct schema id from STRUCT_SCHEMA
     * @param bytes raw bytes
     */
    public record StructValue(long schemaId, byte[] bytes) {
    }
}
