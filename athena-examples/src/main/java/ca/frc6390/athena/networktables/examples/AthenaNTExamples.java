package ca.frc6390.athena.networktables.examples;

import ca.frc6390.athena.networktables.AthenaNT;
import ca.frc6390.athena.networktables.AthenaNTBinding;
import ca.frc6390.athena.networktables.NtScope;

/**
 * Examples for the AthenaNT wrapper and annotation binding flow.
 */
public final class AthenaNTExamples {
    private AthenaNTExamples() {}

    public static void publishDrivetrainTelemetry(boolean enabled, double headingDeg, String mode) {
        AthenaNT.put("Drive/Enabled", enabled);
        AthenaNT.put("Drive/HeadingDeg", headingDeg);
        AthenaNT.put("Drive/Mode", mode);
    }

    public static double readHeadingDegrees(double defaultValue) {
        return AthenaNT.getDouble("Drive/HeadingDeg", defaultValue);
    }

    public static NtScope scoped(String path) {
        return AthenaNT.scope(path);
    }

    public static AthenaNTBinding bind(String scopePath, Object target) {
        return AthenaNT.bind(scopePath, target);
    }

    public static void tickBindings() {
        AthenaNT.tick();
    }
}
