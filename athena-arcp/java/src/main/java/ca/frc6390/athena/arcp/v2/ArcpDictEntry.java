package ca.frc6390.athena.arcp.v2;

/**
 * One signal announced in the dictionary (appendix E1-W, DICTIONARY record).
 *
 * @param signalId stable numeric id used by frames
 * @param path mechanism-tree path, for example {@code robot/turret/angle}
 * @param typeTag wire type tag, see {@link ArcpCodec}
 * @param unit unit label, empty when none
 * @param role one of the {@code ROLE_*} constants
 * @param flags bit set of {@link #FLAG_WRITABLE} and {@link #FLAG_PERSISTED}
 * @param lockGroup lock group simple name, empty when none
 * @param structSchemaId schema id for struct signals, 0 otherwise
 */
public record ArcpDictEntry(
        long signalId,
        String path,
        int typeTag,
        String unit,
        int role,
        int flags,
        String lockGroup,
        long structSchemaId) {

    /** Normative role values (off-season spec, dictionary role table). */
    public static final int ROLE_UNSPECIFIED = 0;
    public static final int ROLE_STATE = 1;
    public static final int ROLE_VALUE = 2;
    public static final int ROLE_CONTROL = 3;
    public static final int ROLE_DEVICE = 4;
    public static final int ROLE_ACTION = 5;
    public static final int ROLE_HOOK = 6;
    /** Boolean lock-state signal at {@code athena/locks/<group>}; true = locked. */
    public static final int ROLE_LOCK = 7;
    public static final int ROLE_SYSTEM = 8;

    public static final int FLAG_WRITABLE = 0x01;
    public static final int FLAG_PERSISTED = 0x02;

    public ArcpDictEntry {
        path = path == null ? "" : path;
        unit = unit == null ? "" : unit;
        lockGroup = lockGroup == null ? "" : lockGroup;
    }

    public boolean writable() {
        return (flags & FLAG_WRITABLE) != 0;
    }

    public boolean persisted() {
        return (flags & FLAG_PERSISTED) != 0;
    }
}
