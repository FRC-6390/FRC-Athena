package ca.frc6390.athena.arcp;

/** Marker kinds (appendix E1-W). Kinds from 100 upward are free for user markers. */
public final class ArcpMarker {

    public static final int BOOT = 1;
    public static final int MATCH_IDENTIFIED = 2;
    public static final int AUTO_START = 3;
    public static final int TELEOP_START = 4;
    public static final int MATCH_END = 5;
    public static final int DISABLE = 6;
    public static final int FAULT = 7;
    public static final int STAGE_CHANGE = 8;
    public static final int CHECK = 9;
    public static final int OVERRIDE = 10;
    public static final int PERSISTED_WRITE = 11;
    public static final int RECORD = 12;
    public static final int TEST_START = 13;
    public static final int BROWNOUT = 14;
    public static final int USER_BASE = 100;

    private ArcpMarker() {
    }
}
