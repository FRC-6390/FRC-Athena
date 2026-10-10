package ca.frc6390.athena.arcp.v2;

import java.util.Objects;

/**
 * Robot decision for a write or action request. Writes reply with {@code {accepted, reason,
 * actualValue}} so the client can restore the displayed value on rejection.
 *
 * @param accepted true when the robot applied the request
 * @param reason rejection reason returned to the client (empty when accepted)
 * @param actualValue value the robot holds after deciding (required)
 */
public record WriteResult(boolean accepted, String reason, Object actualValue) {

    public WriteResult {
        reason = reason == null ? "" : reason;
        Objects.requireNonNull(actualValue, "actualValue");
    }

    /** Accepted; {@code actualValue} is what the robot now holds. */
    public static WriteResult accept(Object actualValue) {
        return new WriteResult(true, "", actualValue);
    }

    /** Rejected with a reason; {@code actualValue} is the value the robot still holds. */
    public static WriteResult reject(String reason, Object actualValue) {
        return new WriteResult(false, reason, actualValue);
    }
}
