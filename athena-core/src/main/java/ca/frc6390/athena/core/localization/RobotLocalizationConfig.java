package ca.frc6390.athena.core.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class RobotLocalizationConfig {
    private StdDevs stateStdDevs;
    private StdDevs visionStdDevs;
    private StdDevs visionMultiStdDevs;
    private HolonomicPidConstants translation;
    private HolonomicPidConstants rotation;
    private AutoPlannerPidAutotunerConfig autoPlannerPidAutotuner;
    private boolean useVision;
    private PoseSpace poseSpace;
    private BackendConfig backend;
    private List<PoseConfig> poseConfigs;
    private List<NamedBoundingBox> boundingBoxes;
    private String autoPoseName;

    public enum PoseSpace {
        TWO_D,
        THREE_D
    }

    public RobotLocalizationConfig(
            double xStd,
            double yStd,
            double thetaStd,
            double vXStd,
            double vYStda,
            double vThetaStd,
            double v2XStd,
            double v2YStda,
            double v2ThetaStd,
            double zStd,
            double vZStd,
            double v2ZStd,
            HolonomicPidConstants translation,
            HolonomicPidConstants rotation,
            boolean useVision,
            PoseSpace poseSpace,
            BackendConfig backend,
            List<PoseConfig> poseConfigs) {
        this.stateStdDevs = new StdDevs(xStd, yStd, zStd, thetaStd);
        this.visionStdDevs = new StdDevs(vXStd, vYStda, vZStd, vThetaStd);
        this.visionMultiStdDevs = new StdDevs(v2XStd, v2YStda, v2ZStd, v2ThetaStd);
        this.translation = translation;
        this.rotation = rotation;
        this.useVision = useVision;
        this.poseSpace = poseSpace;
        this.backend = backend;
        this.poseConfigs = poseConfigs;
        normalize();
    }

    public RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {
        this(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                xStd,
                vXStd,
                vXStd,
                new HolonomicPidConstants(0, 0, 0),
                new HolonomicPidConstants(0, 0, 0),
                true,
                PoseSpace.TWO_D,
                BackendConfig.defaults(),
                List.of());
    }

    public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
        this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
    }

    public RobotLocalizationConfig() {
        this(0.1, 0.1, 0.001);
    }

    public static RobotLocalizationConfig vision(double vXStd, double vYStda, double vThetaStd){
        return create().estimation(e -> e.vision(vXStd, vYStda, vThetaStd).visionEnabled(true));
    }

    public static RobotLocalizationConfig defaults(){
        return new RobotLocalizationConfig();
    }

    public static RobotLocalizationConfig create() {
        return defaults();
    }

    public RobotLocalizationConfig estimation(Consumer<EstimationSection> section) {
        if (section != null) {
            section.accept(new EstimationSection());
        }
        return this;
    }

    public EstimationSection estimation() {
        return new EstimationSection();
    }

    public RobotLocalizationConfig planner(Consumer<PlannerSection> section) {
        if (section != null) {
            section.accept(new PlannerSection());
        }
        return this;
    }

    public PlannerSection planner() {
        return new PlannerSection();
    }

    public RobotLocalizationConfig backendConfig(Consumer<BackendSection> section) {
        if (section != null) {
            section.accept(new BackendSection());
        }
        return this;
    }

    public BackendSection backendConfig() {
        return new BackendSection();
    }

    public RobotLocalizationConfig poses(Consumer<PosesSection> section) {
        if (section != null) {
            section.accept(new PosesSection());
        }
        return this;
    }

    public PosesSection poses() {
        return new PosesSection();
    }

    public RobotLocalizationConfig config(Consumer<ConfigSection> section) {
        if (section != null) {
            section.accept(new ConfigSection());
        }
        return this;
    }

    public ConfigSection config() {
        return new ConfigSection();
    }

    public final class ConfigSection {
        public ConfigSection autoPlannerPid(
                double tP,
                double tI,
                double tD,
                double rP,
                double rI,
                double rD) {
            applyAutoPlannerPID(tP, tI, tD, rP, rI, rD);
            return this;
        }

        public ConfigSection autoPlannerPid(
                HolonomicPidConstants translationConstants,
                HolonomicPidConstants rotationConstants) {
            applyAutoPlannerPID(translationConstants, rotationConstants);
            return this;
        }

        public ConfigSection autoPlannerPid(Consumer<AutoPlannerPidSection> section) {
            AutoPlannerPidSection spec = AutoPlannerPidSection.create(
                    translation,
                    rotation,
                    autoPlannerPidAutotuner());
            if (section != null) {
                section.accept(spec);
            }
            applyAutoPlannerPID(spec.translationConstants(), spec.rotationConstants(), spec.autotunerConfig());
            return this;
        }

        public ConfigSection vision(double vXStd, double vYStd, double vThetaStd) {
            applyVision(vXStd, vYStd, vThetaStd);
            return this;
        }

        public ConfigSection vision(double vXStd, double vYStd, double vZStd, double vThetaStd) {
            applyVision(vXStd, vYStd, vZStd, vThetaStd);
            return this;
        }

        public ConfigSection visionMultitag(double vXStd, double vYStd, double vThetaStd) {
            applyVisionMultitag(vXStd, vYStd, vThetaStd);
            return this;
        }

        public ConfigSection visionMultitag(double vXStd, double vYStd, double vZStd, double vThetaStd) {
            applyVisionMultitag(vXStd, vYStd, vZStd, vThetaStd);
            return this;
        }

        public ConfigSection visionEnabled(boolean enabled) {
            applyVisionEnabled(enabled);
            return this;
        }

        public ConfigSection poseSpace(PoseSpace space) {
            applyPoseSpace(space);
            return this;
        }

        public ConfigSection zStd(double value) {
            applyZStd(value);
            return this;
        }

        public ConfigSection visionZStd(double value) {
            applyVisionZStd(value);
            return this;
        }

        public ConfigSection visionMultiZStd(double value) {
            applyVisionMultiZStd(value);
            return this;
        }

        public ConfigSection use3d() {
            RobotLocalizationConfig.this.use3d();
            return this;
        }

        public ConfigSection use2d() {
            RobotLocalizationConfig.this.use2d();
            return this;
        }

        public ConfigSection backend(BackendConfig backendConfig) {
            applyBackend(backendConfig);
            return this;
        }

        public ConfigSection poseConfigs(List<PoseConfig> configs) {
            applyPoseConfigs(configs);
            return this;
        }

        public ConfigSection pose(PoseConfig poseConfig) {
            addPoseConfig(poseConfig);
            return this;
        }

        public ConfigSection boundingBoxes(List<NamedBoundingBox> boxes) {
            applyBoundingBoxes(boxes);
            return this;
        }

        public ConfigSection boundingBox(String name, PoseBoundingBox2d box) {
            addBoundingBox(name, box);
            return this;
        }

        public ConfigSection boundingBox(String name, Translation2d cornerA, Translation2d cornerB) {
            addBoundingBox(name, cornerA, cornerB);
            return this;
        }

        public ConfigSection autoPoseName(String name) {
            applyAutoPoseName(name);
            return this;
        }

        public ConfigSection slipYawRateThreshold(double value) {
            applySlipYawRateThreshold(value);
            return this;
        }

        public ConfigSection slipYawRateDisagreement(double value) {
            applySlipYawRateDisagreement(value);
            return this;
        }

        public ConfigSection slipAccelThreshold(double value) {
            applySlipAccelThreshold(value);
            return this;
        }

        public ConfigSection slipAccelDisagreement(double value) {
            applySlipAccelDisagreement(value);
            return this;
        }

        public ConfigSection slipHoldSeconds(double value) {
            applySlipHoldSeconds(value);
            return this;
        }

        public ConfigSection slipVisionStdDevScale(double value) {
            applySlipVisionStdDevScale(value);
            return this;
        }

        public ConfigSection slipProcessStdDevScale(double value) {
            applySlipProcessStdDevScale(value);
            return this;
        }

        public ConfigSection imuStrategy(BackendConfig.ImuStrategy strategy) {
            applyImuStrategy(strategy);
            return this;
        }

        public ConfigSection visionStrategy(BackendConfig.VisionStrategy strategy) {
            applyVisionStrategy(strategy);
            return this;
        }

        public ConfigSection visionFusionMaxSeparationSeconds(double value) {
            applyVisionFusionMaxSeparationSeconds(value);
            return this;
        }

        public ConfigSection visionFusionMinWeight(double value) {
            applyVisionFusionMinWeight(value);
            return this;
        }

        public ConfigSection visionFusionDistanceWeight(double value) {
            applyVisionFusionDistanceWeight(value);
            return this;
        }

        public ConfigSection visionFusionLatencyWeight(double value) {
            applyVisionFusionLatencyWeight(value);
            return this;
        }

        public ConfigSection visionFusionConfidenceExponent(double value) {
            applyVisionFusionConfidenceExponent(value);
            return this;
        }

        public ConfigSection poseJumpMeters(double value) {
            applyPoseJumpMeters(value);
            return this;
        }

        public ConfigSection poseJumpHoldSeconds(double value) {
            applyPoseJumpHoldSeconds(value);
            return this;
        }

        public ConfigSection poseJumpAgreementMeters(double value) {
            applyPoseJumpAgreementMeters(value);
            return this;
        }
    }

    public final class EstimationSection {
        public EstimationSection vision(double vXStd, double vYStd, double vThetaStd) {
            applyVision(vXStd, vYStd, vThetaStd);
            return this;
        }

        public EstimationSection vision(double vXStd, double vYStd, double vZStd, double vThetaStd) {
            applyVision(vXStd, vYStd, vZStd, vThetaStd);
            return this;
        }

        public EstimationSection visionMultitag(double vXStd, double vYStd, double vThetaStd) {
            applyVisionMultitag(vXStd, vYStd, vThetaStd);
            return this;
        }

        public EstimationSection visionMultitag(double vXStd, double vYStd, double vZStd, double vThetaStd) {
            applyVisionMultitag(vXStd, vYStd, vZStd, vThetaStd);
            return this;
        }

        public EstimationSection visionEnabled(boolean enabled) {
            applyVisionEnabled(enabled);
            return this;
        }

        public EstimationSection poseSpace(PoseSpace space) {
            applyPoseSpace(space);
            return this;
        }

        public EstimationSection use3d() {
            RobotLocalizationConfig.this.use3d();
            return this;
        }

        public EstimationSection use2d() {
            RobotLocalizationConfig.this.use2d();
            return this;
        }

        public EstimationSection zStd(double value) {
            applyZStd(value);
            return this;
        }

        public EstimationSection visionZStd(double value) {
            applyVisionZStd(value);
            return this;
        }

        public EstimationSection visionMultiZStd(double value) {
            applyVisionMultiZStd(value);
            return this;
        }
    }

    public final class PlannerSection {
        public PlannerSection autoPlannerPid(
                double tP,
                double tI,
                double tD,
                double rP,
                double rI,
                double rD) {
            applyAutoPlannerPID(tP, tI, tD, rP, rI, rD);
            return this;
        }

        public PlannerSection autoPlannerPid(
                HolonomicPidConstants translationConstants,
                HolonomicPidConstants rotationConstants) {
            applyAutoPlannerPID(translationConstants, rotationConstants);
            return this;
        }

        public PlannerSection autoPlannerPid(Consumer<AutoPlannerPidSection> section) {
            AutoPlannerPidSection spec = AutoPlannerPidSection.create(
                    translation,
                    rotation,
                    autoPlannerPidAutotuner());
            if (section != null) {
                section.accept(spec);
            }
            applyAutoPlannerPID(spec.translationConstants(), spec.rotationConstants(), spec.autotunerConfig());
            return this;
        }
    }

    public static final class AutoPlannerPidSection {
        private final PidAxisSection translation;
        private final PidAxisSection rotation;
        private AutoPlannerPidAutotunerConfig autotuner;

        private AutoPlannerPidSection(
                HolonomicPidConstants translation,
                HolonomicPidConstants rotation,
                AutoPlannerPidAutotunerConfig autotuner) {
            this.translation = new PidAxisSection(translation);
            this.rotation = new PidAxisSection(rotation);
            this.autotuner = autotuner != null
                    ? autotuner
                    : AutoPlannerPidAutotunerConfig.defaults();
        }

        static AutoPlannerPidSection create(
                HolonomicPidConstants translation,
                HolonomicPidConstants rotation,
                AutoPlannerPidAutotunerConfig autotuner) {
            return new AutoPlannerPidSection(
                    translation != null ? translation : new HolonomicPidConstants(0.0, 0.0, 0.0, 0.0),
                    rotation != null ? rotation : new HolonomicPidConstants(0.0, 0.0, 0.0, 0.0),
                    autotuner);
        }

        public PidAxisSection translation() {
            return translation;
        }

        public PidAxisSection rotation() {
            return rotation;
        }

        public AutoPlannerPidSection translation(Consumer<PidAxisSection> section) {
            if (section != null) {
                section.accept(translation);
            }
            return this;
        }

        public AutoPlannerPidSection rotation(Consumer<PidAxisSection> section) {
            if (section != null) {
                section.accept(rotation);
            }
            return this;
        }

        public AutoPlannerPidSection autotuner() {
            autotuner = autotuner.withEnabled(true);
            return this;
        }

        public AutoPlannerPidSection autotuner(AutoPlannerPidAutotunerProgram program) {
            autotuner = autotuner.withEnabled(true).withProgram(program);
            return this;
        }

        public AutoPlannerPidSection autotunerConfig(Consumer<AutoPlannerPidAutotunerSection> section) {
            AutoPlannerPidAutotunerSection builder = AutoPlannerPidAutotunerSection.from(autotuner);
            if (section != null) {
                section.accept(builder);
            }
            autotuner = builder.build();
            return this;
        }

        HolonomicPidConstants translationConstants() {
            return translation.toConstants();
        }

        HolonomicPidConstants rotationConstants() {
            return rotation.toConstants();
        }

        AutoPlannerPidAutotunerConfig autotunerConfig() {
            return autotuner;
        }
    }

    public static final class PidAxisSection {
        private double kP;
        private double kI;
        private double kD;
        private double iZone;

        private PidAxisSection(HolonomicPidConstants initial) {
            HolonomicPidConstants resolved = initial != null
                    ? initial
                    : new HolonomicPidConstants(0.0, 0.0, 0.0, 0.0);
            this.kP = resolved.kP();
            this.kI = resolved.kI();
            this.kD = resolved.kD();
            this.iZone = resolved.iZone();
        }

        public PidAxisSection kp(double kP) {
            this.kP = kP;
            return this;
        }

        public PidAxisSection ki(double kI) {
            this.kI = kI;
            return this;
        }

        public PidAxisSection kd(double kD) {
            this.kD = kD;
            return this;
        }

        public PidAxisSection iZone(double iZone) {
            this.iZone = iZone;
            return this;
        }

        HolonomicPidConstants toConstants() {
            return new HolonomicPidConstants(kP, kI, kD, iZone);
        }
    }

    public static final class AutoPlannerPidAutotunerSection {
        private boolean enabled;
        private String dashboardPath;
        private AutoPlannerPidAutotunerProgram program;

        private AutoPlannerPidAutotunerSection(
                boolean enabled,
                String dashboardPath,
                AutoPlannerPidAutotunerProgram program) {
            this.enabled = enabled;
            this.dashboardPath = dashboardPath;
            this.program = program;
        }

        static AutoPlannerPidAutotunerSection from(AutoPlannerPidAutotunerConfig config) {
            AutoPlannerPidAutotunerConfig resolved = config != null
                    ? config
                    : AutoPlannerPidAutotunerConfig.defaults();
            return new AutoPlannerPidAutotunerSection(
                    resolved.enabled(),
                    resolved.dashboardPath(),
                    resolved.program());
        }

        public AutoPlannerPidAutotunerSection enabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }

        public AutoPlannerPidAutotunerSection dashboardPath(String dashboardPath) {
            this.dashboardPath = dashboardPath;
            return this;
        }

        public AutoPlannerPidAutotunerSection program(AutoPlannerPidAutotunerProgram program) {
            this.program = program;
            return this;
        }

        AutoPlannerPidAutotunerConfig build() {
            return new AutoPlannerPidAutotunerConfig(enabled, dashboardPath, program);
        }
    }

    public record AutoPlannerPidAutotunerConfig(
            boolean enabled,
            String dashboardPath,
            AutoPlannerPidAutotunerProgram program) {
        private static final String DEFAULT_DASHBOARD_PATH = "Athena/Localization/AutoPlannerPidAutotuner";

        public AutoPlannerPidAutotunerConfig {
            dashboardPath = normalizeDashboardPath(dashboardPath);
        }

        private static String normalizeDashboardPath(String dashboardPath) {
            if (dashboardPath == null) {
                return DEFAULT_DASHBOARD_PATH;
            }
            String trimmed = dashboardPath.trim();
            return trimmed.isEmpty() ? DEFAULT_DASHBOARD_PATH : trimmed;
        }

        public static AutoPlannerPidAutotunerConfig defaults() {
            return new AutoPlannerPidAutotunerConfig(false, DEFAULT_DASHBOARD_PATH, null);
        }

        public AutoPlannerPidAutotunerConfig withEnabled(boolean enabled) {
            return new AutoPlannerPidAutotunerConfig(enabled, dashboardPath, program);
        }

        public AutoPlannerPidAutotunerConfig withDashboardPath(String dashboardPath) {
            return new AutoPlannerPidAutotunerConfig(enabled, dashboardPath, program);
        }

        public AutoPlannerPidAutotunerConfig withProgram(AutoPlannerPidAutotunerProgram program) {
            return new AutoPlannerPidAutotunerConfig(enabled, dashboardPath, program);
        }
    }

    public final class BackendSection {
        public BackendSection config(BackendConfig backendConfig) {
            applyBackend(backendConfig);
            return this;
        }

        public BackendSection slipYawRateThreshold(double value) {
            applySlipYawRateThreshold(value);
            return this;
        }

        public BackendSection slipYawRateDisagreement(double value) {
            applySlipYawRateDisagreement(value);
            return this;
        }

        public BackendSection slipAccelThreshold(double value) {
            applySlipAccelThreshold(value);
            return this;
        }

        public BackendSection slipAccelDisagreement(double value) {
            applySlipAccelDisagreement(value);
            return this;
        }

        public BackendSection slipHoldSeconds(double value) {
            applySlipHoldSeconds(value);
            return this;
        }

        public BackendSection slipVisionStdDevScale(double value) {
            applySlipVisionStdDevScale(value);
            return this;
        }

        public BackendSection slipProcessStdDevScale(double value) {
            applySlipProcessStdDevScale(value);
            return this;
        }

        public BackendSection imuStrategy(BackendConfig.ImuStrategy strategy) {
            applyImuStrategy(strategy);
            return this;
        }

        public BackendSection visionStrategy(BackendConfig.VisionStrategy strategy) {
            applyVisionStrategy(strategy);
            return this;
        }

        public BackendSection visionFusionMaxSeparationSeconds(double value) {
            applyVisionFusionMaxSeparationSeconds(value);
            return this;
        }

        public BackendSection visionFusionMinWeight(double value) {
            applyVisionFusionMinWeight(value);
            return this;
        }

        public BackendSection visionFusionDistanceWeight(double value) {
            applyVisionFusionDistanceWeight(value);
            return this;
        }

        public BackendSection visionFusionLatencyWeight(double value) {
            applyVisionFusionLatencyWeight(value);
            return this;
        }

        public BackendSection visionFusionConfidenceExponent(double value) {
            applyVisionFusionConfidenceExponent(value);
            return this;
        }

        public BackendSection poseJumpMeters(double value) {
            applyPoseJumpMeters(value);
            return this;
        }

        public BackendSection poseJumpHoldSeconds(double value) {
            applyPoseJumpHoldSeconds(value);
            return this;
        }

        public BackendSection poseJumpAgreementMeters(double value) {
            applyPoseJumpAgreementMeters(value);
            return this;
        }
    }

    public final class PosesSection {
        public PosesSection poseConfigs(List<PoseConfig> configs) {
            applyPoseConfigs(configs);
            return this;
        }

        public PosesSection pose(PoseConfig poseConfig) {
            addPoseConfig(poseConfig);
            return this;
        }

        public PosesSection boundingBoxes(List<NamedBoundingBox> boxes) {
            applyBoundingBoxes(boxes);
            return this;
        }

        public PosesSection boundingBox(String name, PoseBoundingBox2d box) {
            addBoundingBox(name, box);
            return this;
        }

        public PosesSection boundingBox(String name, Translation2d cornerA, Translation2d cornerB) {
            addBoundingBox(name, cornerA, cornerB);
            return this;
        }

        public PosesSection autoPoseName(String name) {
            applyAutoPoseName(name);
            return this;
        }
    }

    public double xStd() {
        return stateStdDevs.x();
    }

    public double yStd() {
        return stateStdDevs.y();
    }

    public double thetaStd() {
        return stateStdDevs.theta();
    }

    public double vXStd() {
        return visionStdDevs.x();
    }

    public double vYStda() {
        return visionStdDevs.y();
    }

    public double vThetaStd() {
        return visionStdDevs.theta();
    }

    public double v2XStd() {
        return visionMultiStdDevs.x();
    }

    public double v2YStda() {
        return visionMultiStdDevs.y();
    }

    public double v2ThetaStd() {
        return visionMultiStdDevs.theta();
    }

    public double zStd() {
        return stateStdDevs.z();
    }

    public double vZStd() {
        return visionStdDevs.z();
    }

    public double v2ZStd() {
        return visionMultiStdDevs.z();
    }

    public HolonomicPidConstants translation() {
        return translation;
    }

    public HolonomicPidConstants rotation() {
        return rotation;
    }

    public AutoPlannerPidAutotunerConfig autoPlannerPidAutotuner() {
        return autoPlannerPidAutotuner != null
                ? autoPlannerPidAutotuner
                : AutoPlannerPidAutotunerConfig.defaults();
    }

    public boolean useVision() {
        return useVision;
    }

    public PoseSpace poseSpace() {
        return poseSpace;
    }

    public BackendConfig backend() {
        return backend != null ? backend : BackendConfig.defaults();
    }

    public String autoPoseName() {
        return autoPoseName;
    }

    public List<PoseConfig> poseConfigs() {
        return poseConfigs != null ? List.copyOf(poseConfigs) : List.of();
    }

    public List<NamedBoundingBox> boundingBoxes() {
        return boundingBoxes != null ? List.copyOf(boundingBoxes) : List.of();
    }

    private RobotLocalizationConfig applyAutoPlannerPID(double tP, double tI, double tD, double rP, double rI, double rD){
        return applyAutoPlannerPID(new HolonomicPidConstants(tP, tI, tD), new HolonomicPidConstants(rP, rI, rD));
    }

    private RobotLocalizationConfig applyAutoPlannerPID(HolonomicPidConstants translation, HolonomicPidConstants rotation){
        this.translation = translation;
        this.rotation = rotation;
        return this;
    }

    private RobotLocalizationConfig applyAutoPlannerPID(
            HolonomicPidConstants translation,
            HolonomicPidConstants rotation,
            AutoPlannerPidAutotunerConfig autotuner) {
        this.translation = translation;
        this.rotation = rotation;
        this.autoPlannerPidAutotuner = autotuner != null
                ? autotuner
                : AutoPlannerPidAutotunerConfig.defaults();
        return this;
    }

    private RobotLocalizationConfig applyVision(double vXStd, double vYStda, double vThetaStd){
        return applyVision(vXStd, vYStda, vXStd, vThetaStd);
    }

    private RobotLocalizationConfig applyVision(double vXStd, double vYStda, double vZStd, double vThetaStd) {
        this.visionStdDevs = new StdDevs(vXStd, vYStda, vZStd, vThetaStd);
        return this;
    }

    private RobotLocalizationConfig applyVisionMultitag(double v2XStd, double v2YStda, double v2ThetaStd){
        return applyVisionMultitag(v2XStd, v2YStda, v2XStd, v2ThetaStd);
    }

    private RobotLocalizationConfig applyVisionMultitag(double v2XStd, double v2YStda, double v2ZStd, double v2ThetaStd){
        this.visionMultiStdDevs = new StdDevs(v2XStd, v2YStda, v2ZStd, v2ThetaStd);
        return this;
    }

    private RobotLocalizationConfig applyVisionEnabled(boolean useVision){
        this.useVision = useVision;
        return this;
    }

    public Matrix<N3, N1> getStd(){
        return VecBuilder.fill(
                stateStdDevs.x(),
                stateStdDevs.y(),
                Units.degreesToRadians(stateStdDevs.theta()));
    }

    public Matrix<N3, N1> getVisionStd(){
        return VecBuilder.fill(
                visionStdDevs.x(),
                visionStdDevs.y(),
                Units.degreesToRadians(visionStdDevs.theta()));
    }
    public Matrix<N3, N1> getVisionMultitagStd(){
        return VecBuilder.fill(
                visionMultiStdDevs.x(),
                visionMultiStdDevs.y(),
                Units.degreesToRadians(visionMultiStdDevs.theta()));
    }

    public Matrix<N3, N1> getStd2d() {
        return getStd();
    }

    public Matrix<N3, N1> getVisionStd2d() {
        return getVisionStd();
    }

    public Matrix<N3, N1> getVisionMultitagStd2d() {
        return getVisionMultitagStd();
    }

    public Matrix<N4, N1> getStd3d(){
        return VecBuilder.fill(
                stateStdDevs.x(),
                stateStdDevs.y(),
                stateStdDevs.z(),
                Units.degreesToRadians(stateStdDevs.theta()));
    }

    public Matrix<N4, N1> getVisionStd3d(){
        return VecBuilder.fill(
                visionStdDevs.x(),
                visionStdDevs.y(),
                visionStdDevs.z(),
                Units.degreesToRadians(visionStdDevs.theta()));
    }

    public Matrix<N4, N1> getVisionMultitagStd3d(){
        return VecBuilder.fill(
                visionMultiStdDevs.x(),
                visionMultiStdDevs.y(),
                visionMultiStdDevs.z(),
                Units.degreesToRadians(visionMultiStdDevs.theta()));
    }

    private RobotLocalizationConfig applyPoseSpace(PoseSpace poseSpace) {
        this.poseSpace = poseSpace;
        normalize();
        return this;
    }

    private RobotLocalizationConfig applyZStd(double zStd) {
        this.stateStdDevs = new StdDevs(
                stateStdDevs.x(),
                stateStdDevs.y(),
                zStd,
                stateStdDevs.theta());
        return this;
    }

    private RobotLocalizationConfig applyVisionZStd(double vZStd) {
        this.visionStdDevs = new StdDevs(
                visionStdDevs.x(),
                visionStdDevs.y(),
                vZStd,
                visionStdDevs.theta());
        return this;
    }

    private RobotLocalizationConfig applyVisionMultiZStd(double v2ZStd) {
        this.visionMultiStdDevs = new StdDevs(
                visionMultiStdDevs.x(),
                visionMultiStdDevs.y(),
                v2ZStd,
                visionMultiStdDevs.theta());
        return this;
    }

    private RobotLocalizationConfig use3d() {
        return applyPoseSpace(PoseSpace.THREE_D);
    }

    private RobotLocalizationConfig use2d() {
        return applyPoseSpace(PoseSpace.TWO_D);
    }

    private RobotLocalizationConfig applyBackend(BackendConfig backend) {
        this.backend = backend;
        normalize();
        return this;
    }

    private RobotLocalizationConfig applyPoseConfigs(List<PoseConfig> poseConfigs) {
        this.poseConfigs = poseConfigs != null ? new ArrayList<>(poseConfigs) : new ArrayList<>();
        return this;
    }

    private RobotLocalizationConfig addPoseConfig(PoseConfig poseConfig) {
        if (poseConfig == null) {
            return this;
        }
        if (poseConfigs == null) {
            poseConfigs = new ArrayList<>();
        }
        poseConfigs.add(poseConfig);
        return this;
    }

    private RobotLocalizationConfig applyBoundingBoxes(List<NamedBoundingBox> boundingBoxes) {
        this.boundingBoxes = boundingBoxes != null ? new ArrayList<>(boundingBoxes) : new ArrayList<>();
        return this;
    }

    private RobotLocalizationConfig addBoundingBox(String name, PoseBoundingBox2d box) {
        if (box == null) {
            return this;
        }
        if (boundingBoxes == null) {
            boundingBoxes = new ArrayList<>();
        }
        NamedBoundingBox namedBoundingBox = new NamedBoundingBox(name, box);
        for (int i = 0; i < boundingBoxes.size(); i++) {
            NamedBoundingBox existing = boundingBoxes.get(i);
            if (existing != null && existing.name().equals(namedBoundingBox.name())) {
                boundingBoxes.set(i, namedBoundingBox);
                return this;
            }
        }
        boundingBoxes.add(namedBoundingBox);
        return this;
    }

    private RobotLocalizationConfig addBoundingBox(
            String name,
            Translation2d cornerA,
            Translation2d cornerB) {
        if (cornerA == null || cornerB == null) {
            return this;
        }
        return addBoundingBox(name, PoseBoundingBox2d.fromCorners(cornerA, cornerB));
    }

    private RobotLocalizationConfig applyAutoPoseName(String autoPoseName) {
        this.autoPoseName = autoPoseName;
        normalize();
        return this;
    }

    private RobotLocalizationConfig applySlipYawRateThreshold(double slipYawRateThreshold) {
        return applyBackend(backend().withSlipYawRateThreshold(slipYawRateThreshold));
    }

    private RobotLocalizationConfig applySlipYawRateDisagreement(double slipYawRateDisagreement) {
        return applyBackend(backend().withSlipYawRateDisagreement(slipYawRateDisagreement));
    }

    private RobotLocalizationConfig applySlipAccelThreshold(double slipAccelThreshold) {
        return applyBackend(backend().withSlipAccelThreshold(slipAccelThreshold));
    }

    private RobotLocalizationConfig applySlipAccelDisagreement(double slipAccelDisagreement) {
        return applyBackend(backend().withSlipAccelDisagreement(slipAccelDisagreement));
    }

    private RobotLocalizationConfig applySlipHoldSeconds(double slipHoldSeconds) {
        return applyBackend(backend().withSlipHoldSeconds(slipHoldSeconds));
    }

    private RobotLocalizationConfig applySlipVisionStdDevScale(double slipVisionStdDevScale) {
        return applyBackend(backend().withSlipVisionStdDevScale(slipVisionStdDevScale));
    }

    private RobotLocalizationConfig applySlipProcessStdDevScale(double slipProcessStdDevScale) {
        return applyBackend(backend().withSlipProcessStdDevScale(slipProcessStdDevScale));
    }

    private RobotLocalizationConfig applyImuStrategy(BackendConfig.ImuStrategy imuStrategy) {
        return applyBackend(backend().withImuStrategy(imuStrategy));
    }

    private RobotLocalizationConfig applyVisionStrategy(BackendConfig.VisionStrategy visionStrategy) {
        return applyBackend(backend().withVisionStrategy(visionStrategy));
    }

    private RobotLocalizationConfig applyVisionFusionMaxSeparationSeconds(double visionFusionMaxSeparationSeconds) {
        return applyBackend(backend().withVisionFusionMaxSeparationSeconds(visionFusionMaxSeparationSeconds));
    }

    private RobotLocalizationConfig applyVisionFusionMinWeight(double visionFusionMinWeight) {
        return applyBackend(backend().withVisionFusionMinWeight(visionFusionMinWeight));
    }

    private RobotLocalizationConfig applyVisionFusionDistanceWeight(double visionFusionDistanceWeight) {
        return applyBackend(backend().withVisionFusionDistanceWeight(visionFusionDistanceWeight));
    }

    private RobotLocalizationConfig applyVisionFusionLatencyWeight(double visionFusionLatencyWeight) {
        return applyBackend(backend().withVisionFusionLatencyWeight(visionFusionLatencyWeight));
    }

    private RobotLocalizationConfig applyVisionFusionConfidenceExponent(double visionFusionConfidenceExponent) {
        return applyBackend(backend().withVisionFusionConfidenceExponent(visionFusionConfidenceExponent));
    }

    private RobotLocalizationConfig applyPoseJumpMeters(double poseJumpMeters) {
        return applyBackend(backend().withPoseJumpMeters(poseJumpMeters));
    }

    private RobotLocalizationConfig applyPoseJumpHoldSeconds(double poseJumpHoldSeconds) {
        return applyBackend(backend().withPoseJumpHoldSeconds(poseJumpHoldSeconds));
    }

    private RobotLocalizationConfig applyPoseJumpAgreementMeters(double poseJumpAgreementMeters) {
        return applyBackend(backend().withPoseJumpAgreementMeters(poseJumpAgreementMeters));
    }

    public boolean isVisionEnabled() {
        return backend().resolveVisionEnabled(useVision);
    }

    private void normalize() {
        poseSpace = poseSpace != null ? poseSpace : PoseSpace.TWO_D;
        backend = backend != null ? backend : BackendConfig.defaults();
        poseConfigs = poseConfigs != null ? new ArrayList<>(poseConfigs) : new ArrayList<>();
        boundingBoxes = boundingBoxes != null ? new ArrayList<>(boundingBoxes) : new ArrayList<>();
        stateStdDevs = stateStdDevs != null ? stateStdDevs : StdDevs.defaults();
        visionStdDevs = visionStdDevs != null ? visionStdDevs : StdDevs.defaults();
        visionMultiStdDevs = visionMultiStdDevs != null ? visionMultiStdDevs : StdDevs.defaults();
        autoPlannerPidAutotuner = autoPlannerPidAutotuner != null
                ? autoPlannerPidAutotuner
                : AutoPlannerPidAutotunerConfig.defaults();
        autoPoseName = (autoPoseName == null || autoPoseName.isBlank()) ? "field" : autoPoseName;
    }

    public record NamedBoundingBox(String name, PoseBoundingBox2d box) {
        public NamedBoundingBox {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(box, "box");
            String resolved = name.trim();
            if (resolved.isEmpty()) {
                throw new IllegalArgumentException("bounding box name must not be blank");
            }
            if (resolved.indexOf('/') >= 0 || resolved.indexOf('\\') >= 0) {
                throw new IllegalArgumentException("bounding box name must not contain '/' or '\\'");
            }
            name = resolved;
        }
    }

    public record StdDevs(double x, double y, double z, double theta) {
        public StdDevs {
            if (!Double.isFinite(x)) {
                x = 0.0;
            }
            if (!Double.isFinite(y)) {
                y = 0.0;
            }
            if (!Double.isFinite(z)) {
                z = 0.0;
            }
            if (!Double.isFinite(theta)) {
                theta = 0.0;
            }
        }

        public static StdDevs defaults() {
            return new StdDevs(0.9, 0.9, 0.9, 0.9);
        }
    }

    public record BackendConfig(
            ImuStrategy imuStrategy,
            VisionStrategy visionStrategy,
            double slipYawRateThreshold,
            double slipYawRateDisagreement,
            double slipAccelThreshold,
            double slipAccelDisagreement,
            double slipHoldSeconds,
            double slipVisionStdDevScale,
            double slipProcessStdDevScale,
            double visionFusionMaxSeparationSeconds,
            double visionFusionMinWeight,
            double visionFusionDistanceWeight,
            double visionFusionLatencyWeight,
            double visionFusionConfidenceExponent,
            double poseJumpMeters,
            double poseJumpHoldSeconds,
            double poseJumpAgreementMeters) {

        public enum ImuStrategy {
            VIRTUAL_AXES,
            RAW_YAW
        }

        public enum VisionStrategy {
            DEFAULT,
            ENABLED,
            DISABLED,
            MULTI
        }

        public static BackendConfig defaults() {
            return new BackendConfig(
                    ImuStrategy.VIRTUAL_AXES,
                    VisionStrategy.DEFAULT,
                    2.0,
                    1.0,
                    1.5,
                    1.0,
                    0.25,
                    0.7,
                    1.5,
                    0.04,
                    1e-3,
                    0.25,
                    1.0,
                    1.0,
                    2.0,
                    0.25,
                    1.0);
        }

        public BackendConfig withImuStrategy(ImuStrategy imuStrategy) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withVisionStrategy(VisionStrategy visionStrategy) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipYawRateThreshold(double slipYawRateThreshold) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipYawRateDisagreement(double slipYawRateDisagreement) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipAccelThreshold(double slipAccelThreshold) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipAccelDisagreement(double slipAccelDisagreement) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipHoldSeconds(double slipHoldSeconds) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipVisionStdDevScale(double slipVisionStdDevScale) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withSlipProcessStdDevScale(double slipProcessStdDevScale) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withVisionFusionMaxSeparationSeconds(double visionFusionMaxSeparationSeconds) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withVisionFusionMinWeight(double visionFusionMinWeight) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withVisionFusionDistanceWeight(double visionFusionDistanceWeight) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withVisionFusionLatencyWeight(double visionFusionLatencyWeight) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withVisionFusionConfidenceExponent(double visionFusionConfidenceExponent) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withPoseJumpMeters(double poseJumpMeters) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withPoseJumpHoldSeconds(double poseJumpHoldSeconds) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public BackendConfig withPoseJumpAgreementMeters(double poseJumpAgreementMeters) {
            return new BackendConfig(
                    imuStrategy,
                    visionStrategy,
                    slipYawRateThreshold,
                    slipYawRateDisagreement,
                    slipAccelThreshold,
                    slipAccelDisagreement,
                    slipHoldSeconds,
                    slipVisionStdDevScale,
                    slipProcessStdDevScale,
                    visionFusionMaxSeparationSeconds,
                    visionFusionMinWeight,
                    visionFusionDistanceWeight,
                    visionFusionLatencyWeight,
                    visionFusionConfidenceExponent,
                    poseJumpMeters,
                    poseJumpHoldSeconds,
                    poseJumpAgreementMeters);
        }

        public boolean resolveVisionEnabled(boolean configuredVisionEnabled) {
            return switch (visionStrategy) {
                case ENABLED, MULTI -> true;
                case DISABLED -> false;
                case DEFAULT -> configuredVisionEnabled;
            };
        }

        public boolean useMultiVision() {
            return visionStrategy == VisionStrategy.MULTI;
        }

        public BackendConfig {
            imuStrategy = imuStrategy != null ? imuStrategy : ImuStrategy.VIRTUAL_AXES;
            visionStrategy = visionStrategy != null ? visionStrategy : VisionStrategy.DEFAULT;
            if (!Double.isFinite(slipYawRateThreshold) || slipYawRateThreshold <= 0.0) {
                slipYawRateThreshold = 2.0;
            }
            if (!Double.isFinite(slipYawRateDisagreement) || slipYawRateDisagreement <= 0.0) {
                slipYawRateDisagreement = 1.0;
            }
            if (!Double.isFinite(slipAccelThreshold) || slipAccelThreshold <= 0.0) {
                slipAccelThreshold = 1.5;
            }
            if (!Double.isFinite(slipAccelDisagreement) || slipAccelDisagreement <= 0.0) {
                slipAccelDisagreement = 1.0;
            }
            if (!Double.isFinite(slipHoldSeconds) || slipHoldSeconds <= 0.0) {
                slipHoldSeconds = 0.25;
            }
            if (!Double.isFinite(slipVisionStdDevScale) || slipVisionStdDevScale <= 0.0) {
                slipVisionStdDevScale = 0.7;
            }
            if (!Double.isFinite(slipProcessStdDevScale) || slipProcessStdDevScale <= 0.0) {
                slipProcessStdDevScale = 1.5;
            }
            if (!Double.isFinite(visionFusionMaxSeparationSeconds) || visionFusionMaxSeparationSeconds <= 0.0) {
                visionFusionMaxSeparationSeconds = 0.04;
            }
            if (!Double.isFinite(visionFusionMinWeight) || visionFusionMinWeight <= 0.0) {
                visionFusionMinWeight = 1e-3;
            }
            if (!Double.isFinite(visionFusionDistanceWeight) || visionFusionDistanceWeight < 0.0) {
                visionFusionDistanceWeight = 0.25;
            }
            if (!Double.isFinite(visionFusionLatencyWeight) || visionFusionLatencyWeight < 0.0) {
                visionFusionLatencyWeight = 1.0;
            }
            if (!Double.isFinite(visionFusionConfidenceExponent) || visionFusionConfidenceExponent < 0.0) {
                visionFusionConfidenceExponent = 1.0;
            }
            if (!Double.isFinite(poseJumpMeters) || poseJumpMeters <= 0.0) {
                poseJumpMeters = 2.0;
            }
            if (!Double.isFinite(poseJumpHoldSeconds) || poseJumpHoldSeconds <= 0.0) {
                poseJumpHoldSeconds = 0.25;
            }
            if (!Double.isFinite(poseJumpAgreementMeters) || poseJumpAgreementMeters <= 0.0) {
                poseJumpAgreementMeters = poseJumpMeters;
            }
        }
    }
}
