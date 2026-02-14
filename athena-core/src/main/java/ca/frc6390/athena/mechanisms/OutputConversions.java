package ca.frc6390.athena.mechanisms;

/**
 * Shared helpers for converting between percent/voltage output spaces.
 */
public final class OutputConversions {
    private OutputConversions() {}

    public static double percentToVolts(double percent, double batteryVoltage) {
        return percent * batteryVoltage;
    }

    public static double voltsToPercent(double volts, double batteryVoltage) {
        if (!Double.isFinite(batteryVoltage) || Math.abs(batteryVoltage) < 1e-9) {
            return 0.0;
        }
        return volts / batteryVoltage;
    }

    public static double convert(OutputType from, OutputType to, double value, double batteryVoltage) {
        OutputType resolvedFrom = from != null ? from : OutputType.PERCENT;
        OutputType resolvedTo = to != null ? to : resolvedFrom;
        if (resolvedFrom == resolvedTo) {
            return value;
        }
        if (resolvedFrom == OutputType.PERCENT && resolvedTo == OutputType.VOLTAGE) {
            return percentToVolts(value, batteryVoltage);
        }
        if (resolvedFrom == OutputType.VOLTAGE && resolvedTo == OutputType.PERCENT) {
            return voltsToPercent(value, batteryVoltage);
        }
        throw new IllegalArgumentException(
                "Unsupported output conversion from " + resolvedFrom + " to " + resolvedTo);
    }

    public static double toMechanismOutput(
            OutputType mechanismOutputType,
            OutputType valueType,
            double value,
            double batteryVoltage) {
        OutputType resolvedTarget = mechanismOutputType != null ? mechanismOutputType : OutputType.PERCENT;
        OutputType resolvedSource = valueType != null ? valueType : resolvedTarget;
        if (resolvedSource == resolvedTarget
                || resolvedSource == OutputType.POSITION
                || resolvedSource == OutputType.VELOCITY) {
            return value;
        }
        return convert(resolvedSource, resolvedTarget, value, batteryVoltage);
    }
}
