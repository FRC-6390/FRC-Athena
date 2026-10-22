package ca.frc6390.athena.robot.replay;

/** Child process used by ReplayBufferTest: the historical full-buffer repro needs a real 96 MiB heap. */
public final class ReplayBackfillHeapProbe {
    private ReplayBackfillHeapProbe() {
    }

    public static void main(String[] args) throws Exception {
        if (args.length > 0 && args[0].equals("writer")) {
            slowWriter();
            return;
        }
        ReplayBuffer buffer = new ReplayBuffer(48L * 1024 * 1024);
        for (int i = 0; i < 768; i++) {
            buffer.add(new ReplayBuffer.Segment(new byte[64 * 1024], i, i, i * 0.02, i * 0.02));
        }
        var groups = buffer.backfill(0, Long.MAX_VALUE);
        long encoded = groups.stream().mapToLong(group -> ReplayBuffer.encodedSize(group.length)).sum();
        if (encoded > ReplayStreamSink.DEFAULT_BACKFILL_BYTES || groups.size() != 15) {
            throw new AssertionError("Backfill exceeds its byte budget: " + encoded);
        }
        System.out.println("history=" + buffer.bytes() + " boundedBytes=" + encoded
                + " heap=" + Runtime.getRuntime().maxMemory());
    }

    private static void slowWriter() throws Exception {
        long budget = 48L * 1024 * 1024;
        var segments = new java.util.ArrayList<byte[]>();
        for (int i = 0; i < 768; i++) segments.add(new byte[64 * 1024]);
        java.util.concurrent.CountDownLatch started = new java.util.concurrent.CountDownLatch(1);
        java.util.concurrent.CountDownLatch release = new java.util.concurrent.CountDownLatch(1);
        java.nio.file.Path directory = java.nio.file.Files.createTempDirectory("athena-writer-heap-");
        ReplayWriter writer = new ReplayWriter(ca.frc6390.athena.robot.RuntimeFailureReporter.stderr(), name -> { }, job -> {
            started.countDown();
            try {
                release.await(10, java.util.concurrent.TimeUnit.SECONDS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
        }, budget + 1024 * 1024);
        java.nio.file.Path target = directory.resolve("large.athrp");
        try {
            writer.submit(new ReplayWriter.Job(target, java.util.Map.of(), new byte[0], segments, "held"));
            segments = null;
            if (!started.await(2, java.util.concurrent.TimeUnit.SECONDS)) throw new AssertionError("writer did not start");
            ReplayBuffer live = new ReplayBuffer(budget);
            for (int i = 0; i < 1024; i++) {
                live.add(new ReplayBuffer.Segment(new byte[64 * 1024], i, i, i * 0.02, i * 0.02),
                        Math.max(0, budget - writer.retainedBytes()));
            }
            if (live.bytes() + writer.retainedBytes() > budget + 64 * 1024) {
                throw new AssertionError("writer and live ring exceeded their shared budget");
            }
            System.out.println("writerBytes=" + writer.retainedBytes() + " liveBytes=" + live.bytes()
                    + " evictions=" + live.evictedSegments() + " heap=" + Runtime.getRuntime().maxMemory());
            release.countDown();
            if (!writer.awaitIdle(java.time.Duration.ofSeconds(10))) throw new AssertionError("writer did not drain");
        } finally {
            release.countDown();
            writer.close();
            java.nio.file.Files.deleteIfExists(target);
            java.nio.file.Files.deleteIfExists(directory);
        }
    }

}
