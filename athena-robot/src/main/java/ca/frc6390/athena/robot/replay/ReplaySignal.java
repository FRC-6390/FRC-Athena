package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.runtime.wire.Wire;

/**
 * One dictionary entry of a replay stream or file (appendix E1-W, DICTIONARY record).
 *
 * @param id stable integer signal id, starting at 1 (0 means "no signal")
 * @param path slash-separated path such as {@code hardware/motor/intake/positionRotations}
 * @param typeTag wire type tag from {@link Wire}
 * @param unit display unit, may be empty
 * @param role one of the {@code ROLE_*} constants
 * @param flags bit set; bit0 writable, bit1 persisted
 * @param lockGroup lock group, may be empty
 * @param structSchemaId struct schema id, 0 when not a struct
 */
public record ReplaySignal(
        int id,
        String path,
        int typeTag,
        String unit,
        int role,
        int flags,
        String lockGroup,
        long structSchemaId) {

    public static final int ROLE_UNKNOWN = 0;
    public static final int ROLE_HARDWARE = 1;
    public static final int ROLE_LIFECYCLE = 2;
    public static final int ROLE_DIGITAL_INPUT = 3;
    public static final int ROLE_COMMAND = 4;

    public static final int FLAG_WRITABLE = 1;
    public static final int FLAG_PERSISTED = 2;

    public ReplaySignal {
        path = path == null ? "" : path;
        unit = unit == null ? "" : unit;
        lockGroup = lockGroup == null ? "" : lockGroup;
    }
}
