package ca.frc6390.athena.robot.replay;

/**
 * Replay file retention rules, enforced after each flush. Deletion removes practice files first, then
 * matches (oldest first), unless {@link #keepMatchesOverPractice()} is off, in which case the oldest file
 * goes first. The file just written is never deleted.
 *
 * <p>Instances are immutable. A zero limit means "no limit".</p>
 */
public final class Retention {
    private final long maxTotalBytes;
    private final int maxFiles;
    private final boolean keepMatchesOverPractice;
    private final long minimumFreeDiskBytes;

    private Retention(long maxTotalBytes, int maxFiles, boolean keepMatchesOverPractice, long minimumFreeDiskBytes) {
        this.maxTotalBytes = maxTotalBytes;
        this.maxFiles = maxFiles;
        this.keepMatchesOverPractice = keepMatchesOverPractice;
        this.minimumFreeDiskBytes = minimumFreeDiskBytes;
    }

    /** Returns the default retention: 400 MiB total, 60 files, matches kept over practice, 150 MiB free. */
    public static Retention defaults() {
        return new Retention(400L * 1024 * 1024, 60, true, 150L * 1024 * 1024);
    }

    /** Returns retention with no limits. */
    public static Retention unlimited() {
        return new Retention(0, 0, false, 0);
    }

    /**
     * Returns retention limited to a total size.
     *
     * @param bytes maximum total size of replay files in bytes
     * @return retention
     */
    public static Retention maxTotalBytes(long bytes) {
        return unlimited().maxTotalBytesLimit(bytes);
    }

    /**
     * Returns retention limited to a total size in megabytes (2^20 bytes).
     *
     * @param megabytes maximum total size
     * @return retention
     */
    public static Retention maxTotalMegabytes(int megabytes) {
        return maxTotalBytes((long) megabytes * 1024 * 1024);
    }

    /**
     * Returns a copy with a total-size limit.
     *
     * @param bytes maximum total size, 0 for none
     * @return retention
     */
    public Retention maxTotalBytesLimit(long bytes) {
        return new Retention(bytes, maxFiles, keepMatchesOverPractice, minimumFreeDiskBytes);
    }

    /**
     * Returns a copy with a file-count limit.
     *
     * @param files maximum number of replay files, 0 for none
     * @return retention
     */
    public Retention maxFiles(int files) {
        return new Retention(maxTotalBytes, files, keepMatchesOverPractice, minimumFreeDiskBytes);
    }

    /**
     * Returns a copy that deletes practice files before match files.
     *
     * @return retention
     */
    public Retention keepMatchesOverPractice() {
        return new Retention(maxTotalBytes, maxFiles, true, minimumFreeDiskBytes);
    }

    /**
     * Returns a copy that deletes the oldest file regardless of match or practice.
     *
     * @return retention
     */
    public Retention deleteOldestFirst() {
        return new Retention(maxTotalBytes, maxFiles, false, minimumFreeDiskBytes);
    }

    /**
     * Returns a copy that keeps at least this much free disk space.
     *
     * @param bytes minimum usable bytes on the replay volume, 0 for none
     * @return retention
     */
    public Retention minimumFreeDiskBytes(long bytes) {
        return new Retention(maxTotalBytes, maxFiles, keepMatchesOverPractice, bytes);
    }

    /**
     * Returns a copy that keeps at least this much free disk space, in megabytes.
     *
     * @param megabytes minimum free space
     * @return retention
     */
    public Retention minimumFreeDiskMegabytes(int megabytes) {
        return minimumFreeDiskBytes((long) megabytes * 1024 * 1024);
    }

    public long maxTotalBytes() {
        return maxTotalBytes;
    }

    public int maxFiles() {
        return maxFiles;
    }

    public boolean keepsMatchesOverPractice() {
        return keepMatchesOverPractice;
    }

    public long minimumFreeDiskBytes() {
        return minimumFreeDiskBytes;
    }
}
