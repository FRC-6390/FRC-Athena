package ca.frc6390.athena.core.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class AutoPlannerPidAutotuners {
    private AutoPlannerPidAutotuners() {}

    private static final String KEY_RELAY_OMEGA = "RelayOmegaRadPerSec";
    private static final String KEY_ERROR_BAND_DEG = "ErrorBandDeg";
    private static final String KEY_TIMEOUT_SEC = "TimeoutSec";
    private static final String KEY_TARGET_CYCLES = "TargetCycles";
    private static final String KEY_STATUS = "Status";
    private static final String KEY_LIVE_ERROR_DEG = "LiveErrorDeg";
    private static final String KEY_LIVE_OUTPUT = "LiveOmegaCmd";
    private static final String KEY_KU = "Ku";
    private static final String KEY_TU = "TuSec";
    private static final String KEY_AVG_PEAK_DEG = "AveragePeakDeg";
    private static final String KEY_SUGGESTED_KP = "Suggested/Rotation/kP";
    private static final String KEY_SUGGESTED_KI = "Suggested/Rotation/kI";
    private static final String KEY_SUGGESTED_KD = "Suggested/Rotation/kD";

    public static void initializeDashboard(String dashboardPath) {
        String root = sanitizeDashboardPath(dashboardPath);
        putDefaultNumber(key(root, KEY_RELAY_OMEGA), RelayThetaConfig.defaults().relayOmegaRadPerSec());
        putDefaultNumber(key(root, KEY_ERROR_BAND_DEG), RelayThetaConfig.defaults().errorBandDeg());
        putDefaultNumber(key(root, KEY_TIMEOUT_SEC), RelayThetaConfig.defaults().timeoutSec());
        putDefaultNumber(key(root, KEY_TARGET_CYCLES), RelayThetaConfig.defaults().targetCycles());
        putDefaultString(key(root, KEY_STATUS), "Idle");
        putDefaultNumber(key(root, KEY_LIVE_ERROR_DEG), 0.0);
        putDefaultNumber(key(root, KEY_LIVE_OUTPUT), 0.0);
    }

    public static Command relayTheta(AutoPlannerPidAutotunerContext ctx) {
        return relayTheta(ctx, RelayThetaConfig.defaults());
    }

    public static Command relayTheta(AutoPlannerPidAutotunerContext ctx, RelayThetaConfig config) {
        Objects.requireNonNull(ctx, "ctx");
        RelayThetaConfig defaults = config != null ? config.sanitized() : RelayThetaConfig.defaults();
        String root = sanitizeDashboardPath(ctx.dashboardPath());
        initializeDashboard(root);

        final class RelayState {
            private final List<Double> halfCycleDurations = new ArrayList<>();
            private final List<Double> peakErrorsRad = new ArrayList<>();
            private double startSec;
            private double targetHeadingRad;
            private double lastSwitchSec;
            private double peakSinceSwitchRad;
            private int relaySign;
            private int switchCount;
            private int requiredHalfCycles;
            private double relayOmega;
            private double errorBandRad;
            private double timeoutSec;
            private boolean finished;

            private void initialize() {
                relayOmega = clamp(
                        SmartDashboard.getNumber(key(root, KEY_RELAY_OMEGA), defaults.relayOmegaRadPerSec()),
                        0.20,
                        1.50);
                errorBandRad = Math.toRadians(clamp(
                        SmartDashboard.getNumber(key(root, KEY_ERROR_BAND_DEG), defaults.errorBandDeg()),
                        1.0,
                        12.0));
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
                targetHeadingRad = ctx.localization().getFieldPose().getRotation().getRadians();
                lastSwitchSec = Double.NaN;
                peakSinceSwitchRad = 0.0;
                relaySign = 1;
                switchCount = 0;
                halfCycleDurations.clear();
                peakErrorsRad.clear();
                finished = false;

                SmartDashboard.putString(key(root, KEY_STATUS), "Running");
                DriverStation.reportWarning(String.format(
                        Locale.US,
                        "[AutoPlannerPidAutotuner] Start relayOmega=%.2f rad/s band=%.1f deg timeout=%.1fs",
                        relayOmega,
                        Math.toDegrees(errorBandRad),
                        timeoutSec), false);
            }

            private void execute() {
                double elapsedSec = Timer.getFPGATimestamp() - startSec;
                double headingRad = ctx.localization().getFieldPose().getRotation().getRadians();
                double errorRad = MathUtil.angleModulus(targetHeadingRad - headingRad);
                peakSinceSwitchRad = Math.max(peakSinceSwitchRad, Math.abs(errorRad));

                if (errorRad > errorBandRad && relaySign > 0) {
                    switchRelay(-1, elapsedSec);
                } else if (errorRad < -errorBandRad && relaySign < 0) {
                    switchRelay(1, elapsedSec);
                }

                double outputOmega = relaySign * relayOmega;
                ctx.output(new ChassisSpeeds(0.0, 0.0, outputOmega));
                SmartDashboard.putNumber(key(root, KEY_LIVE_ERROR_DEG), Math.toDegrees(errorRad));
                SmartDashboard.putNumber(key(root, KEY_LIVE_OUTPUT), outputOmega);

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
                        peakErrorsRad.add(peakSinceSwitchRad);
                    }
                }
                lastSwitchSec = elapsedSec;
                peakSinceSwitchRad = 0.0;
                relaySign = newSign;
            }

            private boolean isFinished() {
                return finished;
            }

            private void end(boolean interrupted) {
                ctx.stopOutput();
                SmartDashboard.putNumber(key(root, KEY_LIVE_OUTPUT), 0.0);

                if (halfCycleDurations.size() < 4 || peakErrorsRad.isEmpty()) {
                    String status = interrupted
                            ? "Interrupted - no estimate"
                            : "Insufficient oscillation samples";
                    SmartDashboard.putString(key(root, KEY_STATUS), status);
                    DriverStation.reportWarning("[AutoPlannerPidAutotuner] " + status, false);
                    return;
                }

                double avgHalfCycle = halfCycleDurations.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
                double avgPeakErrorRad = peakErrorsRad.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
                if (!Double.isFinite(avgHalfCycle) || !Double.isFinite(avgPeakErrorRad) || avgPeakErrorRad < 1e-4) {
                    SmartDashboard.putString(key(root, KEY_STATUS), "Invalid oscillation data");
                    DriverStation.reportWarning("[AutoPlannerPidAutotuner] Invalid oscillation data.", false);
                    return;
                }

                double tuSec = 2.0 * avgHalfCycle;
                double ku = (4.0 * relayOmega) / (Math.PI * avgPeakErrorRad);
                double suggestedKp = 0.35 * ku;
                double suggestedKd = 0.08 * ku * tuSec;

                SmartDashboard.putNumber(key(root, KEY_KU), ku);
                SmartDashboard.putNumber(key(root, KEY_TU), tuSec);
                SmartDashboard.putNumber(key(root, KEY_AVG_PEAK_DEG), Math.toDegrees(avgPeakErrorRad));
                SmartDashboard.putNumber(key(root, KEY_SUGGESTED_KP), suggestedKp);
                SmartDashboard.putNumber(key(root, KEY_SUGGESTED_KI), 0.0);
                SmartDashboard.putNumber(key(root, KEY_SUGGESTED_KD), suggestedKd);

                String status = String.format(
                        Locale.US,
                        "Done Ku=%.3f Tu=%.3fs kP=%.3f kD=%.3f",
                        ku,
                        tuSec,
                        suggestedKp,
                        suggestedKd);
                SmartDashboard.putString(key(root, KEY_STATUS), status);
                DriverStation.reportWarning("[AutoPlannerPidAutotuner] " + status, false);
            }
        }

        RelayState state = new RelayState();
        Subsystem requirement = ctx.requirement() != null ? ctx.requirement() : ctx.localization();
        return new FunctionalCommand(
                state::initialize,
                state::execute,
                state::end,
                state::isFinished,
                requirement);
    }

    private static String sanitizeDashboardPath(String dashboardPath) {
        if (dashboardPath == null) {
            return "Athena/Localization/AutoPlannerPidAutotuner";
        }
        String trimmed = dashboardPath.trim();
        if (trimmed.isEmpty()) {
            return "Athena/Localization/AutoPlannerPidAutotuner";
        }
        return trimmed;
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
        return Math.max(min, Math.min(max, value));
    }

    public record RelayThetaConfig(
            double relayOmegaRadPerSec,
            double errorBandDeg,
            double timeoutSec,
            double targetCycles) {

        public RelayThetaConfig {
            if (!Double.isFinite(relayOmegaRadPerSec)) {
                relayOmegaRadPerSec = 0.6;
            }
            if (!Double.isFinite(errorBandDeg)) {
                errorBandDeg = 3.0;
            }
            if (!Double.isFinite(timeoutSec)) {
                timeoutSec = 12.0;
            }
            if (!Double.isFinite(targetCycles)) {
                targetCycles = 6.0;
            }
        }

        public static RelayThetaConfig defaults() {
            return new RelayThetaConfig(0.6, 3.0, 12.0, 6.0);
        }

        public RelayThetaConfig sanitized() {
            return new RelayThetaConfig(
                    clamp(relayOmegaRadPerSec, 0.20, 1.50),
                    clamp(errorBandDeg, 1.0, 12.0),
                    clamp(timeoutSec, 4.0, 30.0),
                    clamp(targetCycles, 4.0, 12.0));
        }

        public static Builder builder() {
            return new Builder();
        }

        public static final class Builder {
            private double relayOmegaRadPerSec = defaults().relayOmegaRadPerSec();
            private double errorBandDeg = defaults().errorBandDeg();
            private double timeoutSec = defaults().timeoutSec();
            private double targetCycles = defaults().targetCycles();

            public Builder relayOmegaRadPerSec(double relayOmegaRadPerSec) {
                this.relayOmegaRadPerSec = relayOmegaRadPerSec;
                return this;
            }

            public Builder errorBandDeg(double errorBandDeg) {
                this.errorBandDeg = errorBandDeg;
                return this;
            }

            public Builder timeoutSec(double timeoutSec) {
                this.timeoutSec = timeoutSec;
                return this;
            }

            public Builder targetCycles(double targetCycles) {
                this.targetCycles = targetCycles;
                return this;
            }

            public RelayThetaConfig build() {
                return new RelayThetaConfig(relayOmegaRadPerSec, errorBandDeg, timeoutSec, targetCycles).sanitized();
            }
        }
    }
}
