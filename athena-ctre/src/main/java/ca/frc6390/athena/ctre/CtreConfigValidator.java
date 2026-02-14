package ca.frc6390.athena.ctre;

/**
 * Centralized fail-fast validation for CTRE device identity fields.
 */
public final class CtreConfigValidator {
    private static final int MIN_CAN_ID = 0;
    private static final int MAX_CAN_ID = 62;

    private CtreConfigValidator() {}

    public static void validateDeviceIdentity(String deviceType, int id, String canbus) {
        if (id < MIN_CAN_ID || id > MAX_CAN_ID) {
            throw new IllegalArgumentException(
                    deviceType + " CAN ID must be in range [" + MIN_CAN_ID + ", " + MAX_CAN_ID + "], got " + id
                            + " (bus=\"" + canbus + "\").");
        }
        validateCanBus(deviceType, canbus);
    }

    private static void validateCanBus(String deviceType, String canbus) {
        if (canbus == null) {
            return;
        }
        String trimmed = canbus.trim();
        if (!trimmed.equals(canbus)) {
            throw new IllegalArgumentException(
                    deviceType + " CAN bus name contains leading/trailing whitespace: \"" + canbus + "\".");
        }
        if (trimmed.isEmpty()) {
            return;
        }
        for (int i = 0; i < trimmed.length(); i++) {
            if (Character.isWhitespace(trimmed.charAt(i))) {
                throw new IllegalArgumentException(
                        deviceType + " CAN bus name contains whitespace: \"" + canbus + "\".");
            }
        }

    }
}
