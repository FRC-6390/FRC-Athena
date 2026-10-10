package ca.frc6390.athena.arcp.v2;

import java.util.List;

/** Typed control-channel messages (E4). Encoding and decoding live in {@link ArcpCodec}. */
public final class ArcpMessages {

    public static final int STATUS_OK = 0;
    public static final int STATUS_REJECTED = 1;
    public static final int STATUS_ERROR = 2;

    private ArcpMessages() {
    }

    /** One file in a file listing. */
    public record FileInfo(String name, long size, long modifiedUnixSecs) {
    }

    /** Structured reply payload. */
    public sealed interface ReplyData {
        record None() implements ReplyData {
        }

        record Subscription(long subscriptionId) implements ReplyData {
        }

        record Session(long sessionId) implements ReplyData {
        }

        record Files(List<FileInfo> files) implements ReplyData {
        }

        record Chunk(boolean eof, byte[] data) implements ReplyData {
        }
    }

    /** Server-to-client messages. */
    public sealed interface ServerMessage {
        record Reply(long requestId, int status, String reason, ReplyData data) implements ServerMessage {
        }

        record Schema(long requestId, List<ArcpDictEntry> dictionary, List<ArcpStructSchema> schemas)
                implements ServerMessage {
        }

        record TimePong(long requestId, long clientMicros, long serverMicros) implements ServerMessage {
        }

        /** Write reply: {@code {accepted, reason, actualValue}} (off-season spec). */
        record WriteReply(long requestId, boolean accepted, String reason, Object actualValue)
                implements ServerMessage {
        }
    }

    /**
     * Client-to-server request. Fields are used per kind:
     * HELLO (text = client name, number = protocol version, port = UDP port),
     * SUBSCRIBE (text = pattern, rate), UNSUBSCRIBE (id), WRITE (id, value), ACTION_RUN/CANCEL (id),
     * TIME_PING (timestamp), FILE_DOWNLOAD (text = name, offset, maxLength), FILE_DELETE (text).
     */
    public record ClientRequest(
            int kind,
            long requestId,
            String text,
            int number,
            int port,
            long id,
            long offset,
            int maxLength,
            ArcpRate rate,
            Object value,
            long timestamp) {

        private static ClientRequest of(int kind, long requestId) {
            return new ClientRequest(kind, requestId, "", 0, 0, 0L, 0L, 0, null, null, 0L);
        }

        public static ClientRequest hello(long requestId, String clientName, int protocolVersion, int udpPort) {
            return new ClientRequest(ArcpCodec.KIND_HELLO, requestId, clientName, protocolVersion, udpPort,
                    0L, 0L, 0, null, null, 0L);
        }

        public static ClientRequest subscribe(long requestId, String pattern, ArcpRate rate) {
            return new ClientRequest(ArcpCodec.KIND_SUBSCRIBE, requestId, pattern, 0, 0, 0L, 0L, 0, rate, null, 0L);
        }

        public static ClientRequest unsubscribe(long requestId, long subscriptionId) {
            return new ClientRequest(ArcpCodec.KIND_UNSUBSCRIBE, requestId, "", 0, 0, subscriptionId, 0L, 0, null, null, 0L);
        }

        public static ClientRequest write(long requestId, long signalId, Object value) {
            return new ClientRequest(ArcpCodec.KIND_WRITE, requestId, "", 0, 0, signalId, 0L, 0, null, value, 0L);
        }

        public static ClientRequest actionRun(long requestId, long signalId) {
            return of(ArcpCodec.KIND_ACTION_RUN, requestId).withId(signalId);
        }

        public static ClientRequest actionCancel(long requestId, long signalId) {
            return of(ArcpCodec.KIND_ACTION_CANCEL, requestId).withId(signalId);
        }

        public static ClientRequest timePing(long requestId, long clientMicros) {
            return new ClientRequest(ArcpCodec.KIND_TIME_PING, requestId, "", 0, 0, 0L, 0L, 0, null, null, clientMicros);
        }

        public static ClientRequest schemaRequest(long requestId) {
            return of(ArcpCodec.KIND_SCHEMA_REQUEST, requestId);
        }

        public static ClientRequest fileList(long requestId) {
            return of(ArcpCodec.KIND_FILE_LIST, requestId);
        }

        public static ClientRequest fileDownload(long requestId, String name, long offset, int maxLength) {
            return new ClientRequest(ArcpCodec.KIND_FILE_DOWNLOAD, requestId, name, 0, 0, 0L, offset, maxLength, null, null, 0L);
        }

        public static ClientRequest fileDelete(long requestId, String name) {
            return of(ArcpCodec.KIND_FILE_DELETE, requestId).withText(name);
        }

        private ClientRequest withId(long signalId) {
            return new ClientRequest(kind, requestId, text, number, port, signalId, offset, maxLength, rate, value, timestamp);
        }

        private ClientRequest withText(String name) {
            return new ClientRequest(kind, requestId, name, number, port, id, offset, maxLength, rate, value, timestamp);
        }
    }
}
