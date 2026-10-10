package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.List;
import java.util.Map;
import java.util.zip.Deflater;

/**
 * Writes {@code .athrp} files (appendix E1-W): magic, format version, METADATA, DICTIONARY, then
 * COMPRESSED_BLOCK records of about 64 KiB uncompressed (raw DEFLATE, nowrap).
 */
final class ReplayFileWriter {
    static final int BLOCK_TARGET_BYTES = 64 * 1024;

    private ReplayFileWriter() {
    }

    /**
     * Writes a replay file atomically: data goes to a temporary sibling that is then moved into place.
     *
     * @param target destination
     * @param metadata METADATA key/value pairs
     * @param dictionary DICTIONARY records for the file header (may be empty)
     * @param segments self-contained segments in order
     * @throws IOException when writing fails
     */
    static void write(Path target, Map<String, String> metadata, byte[] dictionary, List<byte[]> segments)
            throws IOException {
        Path parent = target.toAbsolutePath().getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }
        Path temporary = target.resolveSibling(target.getFileName() + ".tmp");
        try (OutputStream out = new BufferedOutputStream(Files.newOutputStream(temporary))) {
            out.write(Wire.REPLAY_MAGIC);
            out.write(Wire.REPLAY_FORMAT_VERSION & 0xFF);
            out.write((Wire.REPLAY_FORMAT_VERSION >>> 8) & 0xFF);
            WireWriter header = new WireWriter(512);
            ReplayEncoder.writeMetadata(header, metadata);
            out.write(header.buffer(), 0, header.size());
            if (dictionary.length > 0) {
                out.write(dictionary);
            }
            WireWriter block = new WireWriter(BLOCK_TARGET_BYTES + 4096);
            for (byte[] segment : segments) {
                block.bytes(segment, 0, segment.length);
                if (block.size() >= BLOCK_TARGET_BYTES) {
                    writeBlock(out, block);
                    block.clear();
                }
            }
            if (block.size() > 0) {
                writeBlock(out, block);
            }
        }
        try {
            Files.move(temporary, target, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } catch (IOException atomicUnsupported) {
            Files.move(temporary, target, StandardCopyOption.REPLACE_EXISTING);
        }
    }

    private static void writeBlock(OutputStream out, WireWriter block) throws IOException {
        byte[] compressed = deflate(block.buffer(), block.size());
        WireWriter payload = new WireWriter(compressed.length + 10);
        payload.varint(block.size());
        payload.bytes(compressed, 0, compressed.length);
        WireWriter record = new WireWriter(payload.size() + 12);
        record.record(Wire.RECORD_COMPRESSED_BLOCK, payload);
        out.write(record.buffer(), 0, record.size());
    }

    static byte[] deflate(byte[] input, int length) {
        Deflater deflater = new Deflater(Deflater.DEFAULT_COMPRESSION, true);
        try {
            deflater.setInput(input, 0, length);
            deflater.finish();
            byte[] output = new byte[Math.max(64, length / 2)];
            int size = 0;
            while (!deflater.finished()) {
                if (size == output.length) {
                    output = java.util.Arrays.copyOf(output, output.length * 2);
                }
                size += deflater.deflate(output, size, output.length - size);
            }
            return java.util.Arrays.copyOf(output, size);
        } finally {
            deflater.end();
        }
    }
}
