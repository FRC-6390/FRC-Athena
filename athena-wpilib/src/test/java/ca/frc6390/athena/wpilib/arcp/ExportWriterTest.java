package ca.frc6390.athena.wpilib.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ExportWriterTest {
    @TempDir
    Path temp;

    @Test
    void rapidWritesToOneFileCoalesceToTheLatestContents() throws Exception {
        ExportWriter writer = new ExportWriter();
        try {
            for (int version = 0; version < 500; version++) {
                writer.write(temp, "status.txt", "version " + version);
            }
            assertTrue(writer.awaitIdle(5_000));
            assertEquals("version 499", read("status.txt"));
            assertTrue(!Files.exists(temp.resolve("status.txt.tmp")), "no temporary file is left behind");
        } finally {
            writer.close();
        }
    }

    @Test
    void filesAreWrittenIndependentlyAndCloseFlushesWhatIsQueued() throws Exception {
        ExportWriter writer = new ExportWriter();
        writer.write(temp, "diff.txt", "diff v1");
        writer.write(temp, "pull.txt", "pull v1");
        writer.write(temp, "diff.txt", "diff v2");
        writer.close();

        assertEquals("diff v2", read("diff.txt"));
        assertEquals("pull v1", read("pull.txt"));
    }

    @Test
    void writeAfterCloseIsWrittenOnTheCallingThread() throws Exception {
        ExportWriter writer = new ExportWriter();
        writer.close();
        writer.write(temp, "status.txt", "after close");

        assertEquals("after close", read("status.txt"));
    }

    private String read(String name) throws Exception {
        return Files.readString(temp.resolve(name), StandardCharsets.UTF_8);
    }
}
