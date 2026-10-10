package ca.frc6390.athena.arcp.v2;

import ca.frc6390.athena.arcp.v2.ArcpMessages.FileInfo;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ReplyData;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

/**
 * File service (E6): list, download and delete plain file names in one directory. Names with
 * separators, {@code .}/{@code ..}, NUL or drive prefixes are rejected, and symlinks are not served.
 */
public final class ArcpFileService {

    private final Path root;

    public ArcpFileService(Path root) {
        this.root = root.toAbsolutePath().normalize();
    }

    public List<FileInfo> list() throws IOException {
        List<FileInfo> out = new ArrayList<>();
        try (Stream<Path> entries = Files.list(root)) {
            for (Path p : (Iterable<Path>) entries::iterator) {
                if (!Files.isRegularFile(p, LinkOption.NOFOLLOW_LINKS)) {
                    continue;
                }
                out.add(new FileInfo(p.getFileName().toString(), Files.size(p),
                        Files.getLastModifiedTime(p, LinkOption.NOFOLLOW_LINKS).toMillis() / 1000L));
            }
        }
        out.sort((a, b) -> a.name().compareTo(b.name()));
        return out;
    }

    /** Reads up to {@code maxLength} bytes (capped at {@link ArcpCodec#MAX_FILE_CHUNK}) from {@code offset}. */
    public ReplyData.Chunk download(String name, long offset, int maxLength) throws IOException {
        Path path = checkedPath(name);
        long size = regularFile(path);
        int want = Math.min(Math.max(maxLength, 0), ArcpCodec.MAX_FILE_CHUNK);
        byte[] buf;
        try (RandomAccessFile file = new RandomAccessFile(path.toFile(), "r")) {
            long start = Math.min(Math.max(offset, 0L), size);
            file.seek(start);
            int available = (int) Math.min(want, size - start);
            buf = new byte[available];
            file.readFully(buf);
        }
        long end = Math.max(offset, 0L) + buf.length;
        return new ReplyData.Chunk(end >= size, buf);
    }

    public void delete(String name) throws IOException {
        Path path = checkedPath(name);
        regularFile(path);
        Files.delete(path);
    }

    private Path checkedPath(String name) {
        if (name == null || name.isEmpty() || name.length() > 255 || name.equals(".") || name.equals("..")
                || name.indexOf('/') >= 0 || name.indexOf('\\') >= 0 || name.indexOf('\0') >= 0
                || name.indexOf(':') >= 0) {
            throw new IllegalArgumentException("invalid file name");
        }
        return root.resolve(name);
    }

    private static long regularFile(Path path) throws IOException {
        if (!Files.isRegularFile(path, LinkOption.NOFOLLOW_LINKS)) {
            throw new IOException("no such file");
        }
        return Files.size(path);
    }
}
