package ca.frc6390.athena.robot.replay;

/**
 * Field-management identity of the match being played, from the driver station.
 *
 * @param eventName FMS event name, empty when no FMS information exists
 * @param matchType driver station match type name, such as {@code Qualification}
 * @param matchNumber match number, 0 when unknown
 */
public record ReplayMatch(String eventName, String matchType, int matchNumber) {
    public ReplayMatch {
        eventName = eventName == null ? "" : eventName.trim();
        matchType = matchType == null ? "" : matchType.trim();
    }

    /** Returns the "no FMS information" identity. */
    public static ReplayMatch none() {
        return new ReplayMatch("", "", 0);
    }

    /** Returns true when FMS or driver station information identifies a match. */
    public boolean identified() {
        return !eventName.isEmpty() || matchNumber > 0;
    }

    /** Returns true for matches that are not qualification or elimination matches. */
    public boolean practice() {
        return !identified() || matchType.equals("Practice") || matchType.equals("None") || matchType.isEmpty();
    }

    /** Returns the one-letter type code used in replay file names. */
    public String typeCode() {
        return switch (matchType) {
            case "Practice" -> "p";
            case "Qualification" -> "q";
            case "Elimination" -> "e";
            default -> "m";
        };
    }
}
