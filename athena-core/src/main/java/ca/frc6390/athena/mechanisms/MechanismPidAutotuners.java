package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public final class MechanismPidAutotuners {
    private MechanismPidAutotuners() {}

    private static final String KEY_RELAY_OUTPUT = "RelayOutput";
    private static final String KEY_ERROR_BAND = "ErrorBand";
    private static final String KEY_TIMEOUT_SEC = "TimeoutSec";
    private static final String KEY_TARGET_CYCLES = "TargetCycles";
    private static final String KEY_STATUS = "Status";
    private static final String KEY_LIVE_ERROR = "LiveError";
    private static final String KEY_LIVE_OUTPUT = "LiveOutput";
    private static final String KEY_KU = "Ku";
    private static final String KEY_TU = "TuSec";
    private static final String KEY_AVG_PEAK = "AveragePeakError";
    private static final String KEY_SUGGESTED_KP = "Suggested/kP";
    private static final String KEY_SUGGESTED_KI = "Suggested/kI";
    private static final String KEY_SUGGESTED_KD = "Suggested/kD";

    public static void initializeDashboard(String dashboardPath, OutputType outputType) {
        String root = sanitizeDashboardPath(dashboardPath);
        RelayPositionConfig defaults = RelayPositionConfig.defaultsFor(outputType);
        putDefaultNumber(key(root, KEY_RELAY_OUTPUT), defaults.relayOutput());
        putDefaultNumber(key(root, KEY_ERROR_BAND), defaults.errorBand());
        putDefaultNumber(key(root, KEY_TIMEOUT_SEC), defaults.timeoutSec());
        putDefaultNumber(key(root, KEY_TARGET_CYCLES), defaults.targetCycles());
        putDefaultString(key(root, KEY_STATUS), "Idle");
        putDefaultNumber(key(root, KEY_LIVE_ERROR), 0.0);
        putDefaultNumber(key(root, KEY_LIVE_OUTPUT), 0.0);
    }

    public static Command relayPosition(MechanismPidAutotunerContext ctx) {
        Objects.requireNonNull(ctx, "ctx");
        return relayPosition(ctx, RelayPositionConfig.defaultsFor(ctx.outputType()));
    }

    public static Command relayPosition(MechanismPidAutotunerContext ctx, RelayPositionConfig config) {
        Objects.requireNonNull(ctx, "ctx");
        RelayPositionConfig defaults = config != null
                ? config.sanitized(ctx.outputType())
                : RelayPositionConfig.defaultsFor(ctx.outputType());
        String root = sanitizeDashboardPath(ctx.dashboardPath());
        initializeDashboard(root, ctx.outputType());

        final class RelayState {
            private final List<Double> halfCycleDurations = new ArrayList<>();
            private final List<Double> peakErrors = new ArrayList<>();
            private double startSec;
            private double target;
            private double lastSwitchSec;
            private double peakSinceSwitch;
            private int relaySign;
            private int switchCount;
            private int requiredHalfCycles;
            private double relayOutput;
            private double errorBand;
            private double timeoutSec;
            private boolean finished;
            private boolean valid;

            private void initialize() {
                OutputType outputType = ctx.outputType();
                if (outputType != OutputType.PERCENT && outputType != OutputType.VOLTAGE) {
                    SmartDashboard.putString(key(root, KEY_STATUS), "Unsupported output type: " + outputType);
                    DriverStation.reportWarning(
                            "[MechanismPidAutotuner] Unsupported output type for relay tuning: " + outputType,
                            false);
                    finished = true;
                    valid = false;
                    return;
                }

                relayOutput = sanitizeRelayOutput(outputType,
                        SmartDashboard.getNumber(key(root, KEY_RELAY_OUTPUT), defaults.relayOutput()));
                errorBand = Math.max(1e-6,
                        SmartDashboard.getNumber(key(root, KEY_ERROR_BAND), defaults.errorBand()));
                timeoutSec = clamp(
                        SmartDashboard.getNumber(key(root, KEY_TIMEOUT_SEC), defaults.timeoutSec()),
                        4.0,
                        30.0);
                int targetCycles = (int) Math.round(clamp(
                        SmartDashboard.getNumber(key(root, KEY_TARGET_CYCLES), defaults.targetCycles()),
                        4.0,
                        12.0));
                requiredHalfCycles = Math.max(8, targetCycles * 2);

                startSec = Timer.getFPGATimestamp();
                double requestedSetpoint = ctx.setpoint();
                double measurement = ctx.measurement();
                target = Double.isFinite(requestedSetpoint)
                        ? requestedSetpoint
                        : (Double.isFinite(measurement) ? measurement : 0.0);
                lastSwitchSec = Double.NaN;
                peakSinceSwitch = 0.0;
                relaySign = 1;
                switchCount = 0;
                halfCycleDurations.clear();
                peakErrors.clear();
                finished = false;
                valid = true;

                SmartDashboard.putString(key(root, KEY_STATUS), "Running");
                DriverStation.reportWarning(String.format(
                        Locale.US,
                        "[MechanismPidAutotuner] Start pid=%s relay=%.3f band=%.4f timeout=%.1fs",
                        ctx.pidName(),
                        relayOutput,
                        errorBand,
                        timeoutSec), false);
            }

            private void execute() {
                if (!valid) {
                    finished = true;
                    return;
                }
                double elapsedSec = Timer.getFPGATimestamp() - startSec;
                double measurement = ctx.measurement();
                if (!Double.isFinite(measurement)) {
                    finished = true;
                    SmartDashboard.putString(key(root, KEY_STATUS), "Invalid measurement");
                    DriverStation.reportWarning("[MechanismPidAutotuner] Invalid measurement.", false);
                    return;
                }

                double error = target - measurement;
                peakSinceSwitch = Math.max(peakSinceSwitch, Math.abs(error));

                if (error > errorBand && relaySign > 0) {
                    switchRelay(-1, elapsedSec);
                } else if (error < -errorBand && relaySign < 0) {
                    switchRelay(1, elapsedSec);
                }

                double output = relaySign * relayOutput;
                ctx.output(output);
                SmartDashboard.putNumber(key(root, KEY_LIVE_ERROR), error);
                SmartDashboard.putNumber(key(root, KEY_LIVE_OUTPUT), output);

                if (elapsedSec >= timeoutSec || halfCycleDurations.size() >= requiredHalfCycles) {
                    finished = true;
                }
            }

            private void switchRelay(int newSign, double elapsedSec) {
                switchCount++;
                if (Double.isFinite(lastSwitchSec)) {
                    double halfCycle = elapsedSec - lastSwitchSec;
                    if (switchCount > 2 && halfCycle > 1e-3) {
                        halfCycleDurations.add(halfCycle);
                        peakErrors.add(peakSinceSwitch);
                    }
                }
                lastSwitchSec = elapsedSec;
                peakSinceSwitch = 0.0;
                relaySign = newSign;
            }

            private boolean isFinished() {
                return finished;
            }

            private void end(boolean interrupted) {
                ctx.stopOutput();
                SmartDashboard.putNumber(key(root, KEY_LIVE_OUTPUT), 0.0);
                if (!valid) {
                    return;
                }

                if (halfCycleDurations.size() < 4 || peakErrors.isEmpty()) {
                    String status = interrupted
                            ? "Interrupted - no estimate"
                            : "Insufficient oscillation samples";
                    SmartDashboard.putString(key(root, KEY_STATUS), status);
                    DriverStation.reportWarning("[MechanismPidAutotuner] " + status, false);
                    return;
                }

                double avgHalfCycle = halfCycleDurations.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
                double avgPeakError = peakErrors.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
                if (!Double.isFinite(avgHalfCycle) || !Double.isFinite(avgPeakError) || avgPeakError < 1e-6) {
                    SmartDashboard.putString(key(root, KEY_STATUS), "Invalid oscillation data");
                    DriverStation.reportWarning("[MechanismPidAutotuner] Invalid oscillation data.", false);
                    return;
                }

                double tuSec = 2.0 * avgHalfCycle;
                double ku = (4.0 * Math.abs(relayOutput)) / (Math.PI * avgPeakError);
                double suggestedKp = 0.35 * ku;
                double suggestedKd = 0.08 * ku * tuSec;

                SmartDashboard.putNumber(key(root, KEY_KU), ku);
                SmartDashboard.putNumber(key(root, KEY_TU), tuSec);
                SmartDashboard.putNumber(key(root, KEY_AVG_PEAK), avgPeakError);
                SmartDashboard.putNumber(key(root, KEY_SUGGESTED_KP), suggestedKp);
                SmartDashboard.putNumber(key(root, KEY_SUGGESTED_KI), 0.0);
                SmartDashboard.putNumber(key(root, KEY_SUGGESTED_KD), suggestedKd);

                String status = String.format(
                        Locale.US,
                        "Done pid=%s Ku=%.3f Tu=%.3fs kP=%.3f kD=%.3f",
                        ctx.pidName(),
                        ku,
                        tuSec,
                        suggestedKp,
                        suggestedKd);
                SmartDashboard.putString(key(root, KEY_STATUS), status);
                DriverStation.reportWarning("[MechanismPidAutotuner] " + status, false);
            }
        }

        RelayState state = new RelayState();
        return new FunctionalCommand(
                state::initialize,
                state::execute,
                state::end,
                state::isFinished,
                ctx.mechanism());
    }

    private static String sanitizeDashboardPath(String dashboardPath) {
        if (dashboardPath == null) {
            return "Athena/Mechanisms/PidAutotuner";
        }
        String trimmed = dashboardPath.trim();
        return trimmed.isEmpty() ? "Athena/Mechanisms/PidAutotuner" : trimmed;
    }

    private static String key(String root, String suffix) {
        return root + "/" + suffix;
    }

    private static void putDefaultNumber(String key, double value) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, value));
    }

    private static void putDefaultString(String key, String value) {
        String current = SmartDashboard.getString(key, value);
        SmartDashboard.putString(key, current == null || current.isBlank() ? value : current);
    }

    private static double clamp(double value, double min, double max) {
        if (!Double.isFinite(value)) {
            return min;
        }
        return MathUtil.clamp(value, min, max);
    }

    private static double sanitizeRelayOutput(OutputType outputType, double relayOutput) {
        if (outputType == OutputType.VOLTAGE) {
            return Math.copySign(clamp(Math.abs(relayOutput), 0.5, 6.0), relayOutput >= 0.0 ? 1.0 : -1.0);
        }
        if (outputType == OutputType.PERCENT) {
            return Math.copySign(clamp(Math.abs(relayOutput), 0.05, 0.75), relayOutput >= 0.0 ? 1.0 : -1.0);
        }
        return relayOutput;
    }

    public record RelayPositionConfig(
            double relayOutput,
            double errorBand,
            double timeoutSec,
            double targetCycles) {

        public RelayPositionConfig {
            if (!Double.isFinite(relayOutput)) {
                relayOutput = 0.20;
            }
            if (!Double.isFinite(errorBand)) {
                errorBand = 0.02;
            }
            if (!Double.isFinite(timeoutSec)) {
                timeoutSec = 12.0;
            }
            if (!Double.isFinite(targetCycles)) {
                targetCycles = 6.0;
            }
        }

        public static RelayPositionConfig defaultsFor(OutputType outputType) {
            if (outputType == OutputType.VOLTAGE) {
                return new RelayPositionConfig(1.6, 0.02, 12.0, 6.0);
            }
            return new RelayPositionConfig(0.20, 0.02, 12.0, 6.0);
        }

        public RelayPositionConfig sanitized(OutputType outputType) {
            return new RelayPositionConfig(
                    sanitizeRelayOutput(outputType, relayOutput),
                    clamp(errorBand, 1e-6, 1e9),
                    clamp(timeoutSec, 4.0, 30.0),
                    clamp(targetCycles, 4.0, 12.0));
        }
    }
}
