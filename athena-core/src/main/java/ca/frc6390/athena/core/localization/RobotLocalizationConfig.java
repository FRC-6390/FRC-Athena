package ca.frc6390.athena.core.localization;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
    private boolean useVision;
    private PoseSpace poseSpace;
    private BackendConfig backend;
    private List<PoseConfig> poseConfigs;
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
                BackendConfig.defualt(),
                List.of());
    }

    public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
        this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
    }

    public RobotLocalizationConfig() {
        this(0.1, 0.1, 0.001);
    }

    public static RobotLocalizationConfig vision(double vXStd, double vYStda, double vThetaStd){
        return new RobotLocalizationConfig().setVision(vXStd, vYStda, vThetaStd).setVisionEnabled(true);
    }

    public static RobotLocalizationConfig defualt(){
        return new RobotLocalizationConfig();
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

    public boolean useVision() {
        return useVision;
    }

    public PoseSpace poseSpace() {
        return poseSpace;
    }

    public BackendConfig backend() {
        return backend != null ? backend : BackendConfig.defualt();
    }

    public String autoPoseName() {
        return autoPoseName;
    }

    public List<PoseConfig> poseConfigs() {
        return poseConfigs != null ? List.copyOf(poseConfigs) : List.of();
    }

    public RobotLocalizationConfig setAutoPlannerPID(double tP, double tI, double tD, double rP, double rI, double rD){
        return setAutoPlannerPID(new HolonomicPidConstants(tP, tI, tD), new HolonomicPidConstants(rP, rI, rD));
    }

    public RobotLocalizationConfig setAutoPlannerPID(HolonomicPidConstants translation, HolonomicPidConstants rotation){
        this.translation = translation;
        this.rotation = rotation;
        return this;
    }

    public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vThetaStd){
        return setVision(vXStd, vYStda, vXStd, vThetaStd);
    }

    public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vZStd, double vThetaStd) {
        this.visionStdDevs = new StdDevs(vXStd, vYStda, vZStd, vThetaStd);
        return this;
    }

    public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ThetaStd){
        return setVisionMultitag(v2XStd, v2YStda, v2XStd, v2ThetaStd);
    }

    public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ZStd, double v2ThetaStd){
        this.visionMultiStdDevs = new StdDevs(v2XStd, v2YStda, v2ZStd, v2ThetaStd);
        return this;
    }

    public RobotLocalizationConfig setVisionEnabled(boolean useVision){
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

    public RobotLocalizationConfig setPoseSpace(PoseSpace poseSpace) {
        this.poseSpace = poseSpace;
        normalize();
        return this;
    }

    public RobotLocalizationConfig setZStd(double zStd) {
        this.stateStdDevs = new StdDevs(
                stateStdDevs.x(),
                stateStdDevs.y(),
                zStd,
                stateStdDevs.theta());
        return this;
    }

    public RobotLocalizationConfig setVisionZStd(double vZStd) {
        this.visionStdDevs = new StdDevs(
                visionStdDevs.x(),
                visionStdDevs.y(),
                vZStd,
                visionStdDevs.theta());
        return this;
    }

    public RobotLocalizationConfig setVisionMultiZStd(double v2ZStd) {
        this.visionMultiStdDevs = new StdDevs(
                visionMultiStdDevs.x(),
                visionMultiStdDevs.y(),
                v2ZStd,
                visionMultiStdDevs.theta());
        return this;
    }

    public RobotLocalizationConfig use3d() {
        return setPoseSpace(PoseSpace.THREE_D);
    }

    public RobotLocalizationConfig use2d() {
        return setPoseSpace(PoseSpace.TWO_D);
    }

    public RobotLocalizationConfig setBackend(BackendConfig backend) {
        this.backend = backend;
        normalize();
        return this;
    }

    public RobotLocalizationConfig setPoseConfigs(List<PoseConfig> poseConfigs) {
        this.poseConfigs = poseConfigs != null ? new ArrayList<>(poseConfigs) : new ArrayList<>();
        return this;
    }

    public RobotLocalizationConfig addPoseConfig(PoseConfig poseConfig) {
        if (poseConfig == null) {
            return this;
        }
        if (poseConfigs == null) {
            poseConfigs = new ArrayList<>();
        }
        poseConfigs.add(poseConfig);
        return this;
    }

    public RobotLocalizationConfig setAutoPoseName(String autoPoseName) {
        this.autoPoseName = autoPoseName;
        normalize();
        return this;
    }

    public RobotLocalizationConfig setSlipStrategy(BackendConfig.SlipStrategy slipStrategy) {
        return setBackend(backend().withSlipStrategy(slipStrategy));
    }

    public RobotLocalizationConfig setSlipThreshold(double slipThreshold) {
        return setBackend(backend().withSlipThreshold(slipThreshold));
    }

    public RobotLocalizationConfig setSlipYawRateThreshold(double slipYawRateThreshold) {
        return setBackend(backend().withSlipYawRateThreshold(slipYawRateThreshold));
    }

    public RobotLocalizationConfig setSlipYawRateDisagreement(double slipYawRateDisagreement) {
        return setBackend(backend().withSlipYawRateDisagreement(slipYawRateDisagreement));
    }

    public RobotLocalizationConfig setSlipAccelThreshold(double slipAccelThreshold) {
        return setBackend(backend().withSlipAccelThreshold(slipAccelThreshold));
    }

    public RobotLocalizationConfig setSlipAccelDisagreement(double slipAccelDisagreement) {
        return setBackend(backend().withSlipAccelDisagreement(slipAccelDisagreement));
    }

    public RobotLocalizationConfig setSlipHoldSeconds(double slipHoldSeconds) {
        return setBackend(backend().withSlipHoldSeconds(slipHoldSeconds));
    }

    public RobotLocalizationConfig setSlipVisionStdDevScale(double slipVisionStdDevScale) {
        return setBackend(backend().withSlipVisionStdDevScale(slipVisionStdDevScale));
    }

    public RobotLocalizationConfig setSlipProcessStdDevScale(double slipProcessStdDevScale) {
        return setBackend(backend().withSlipProcessStdDevScale(slipProcessStdDevScale));
    }

    public RobotLocalizationConfig setImuStrategy(BackendConfig.ImuStrategy imuStrategy) {
        return setBackend(backend().withImuStrategy(imuStrategy));
    }

    public RobotLocalizationConfig setVisionStrategy(BackendConfig.VisionStrategy visionStrategy) {
        return setBackend(backend().withVisionStrategy(visionStrategy));
    }

    public RobotLocalizationConfig setVisionFusionMaxSeparationSeconds(double visionFusionMaxSeparationSeconds) {
        return setBackend(backend().withVisionFusionMaxSeparationSeconds(visionFusionMaxSeparationSeconds));
    }

    public RobotLocalizationConfig setVisionFusionMinWeight(double visionFusionMinWeight) {
        return setBackend(backend().withVisionFusionMinWeight(visionFusionMinWeight));
    }

    public RobotLocalizationConfig setVisionFusionDistanceWeight(double visionFusionDistanceWeight) {
        return setBackend(backend().withVisionFusionDistanceWeight(visionFusionDistanceWeight));
    }

    public RobotLocalizationConfig setVisionFusionLatencyWeight(double visionFusionLatencyWeight) {
        return setBackend(backend().withVisionFusionLatencyWeight(visionFusionLatencyWeight));
    }

    public RobotLocalizationConfig setVisionFusionConfidenceExponent(double visionFusionConfidenceExponent) {
        return setBackend(backend().withVisionFusionConfidenceExponent(visionFusionConfidenceExponent));
    }

    public RobotLocalizationConfig setPoseJumpMeters(double poseJumpMeters) {
        return setBackend(backend().withPoseJumpMeters(poseJumpMeters));
    }

    public RobotLocalizationConfig setPoseJumpHoldSeconds(double poseJumpHoldSeconds) {
        return setBackend(backend().withPoseJumpHoldSeconds(poseJumpHoldSeconds));
    }

    public RobotLocalizationConfig setPoseJumpAgreementMeters(double poseJumpAgreementMeters) {
        return setBackend(backend().withPoseJumpAgreementMeters(poseJumpAgreementMeters));
    }

    public boolean isVisionEnabled() {
        return backend().resolveVisionEnabled(useVision);
    }

    private void normalize() {
        poseSpace = poseSpace != null ? poseSpace : PoseSpace.TWO_D;
        backend = backend != null ? backend : BackendConfig.defualt();
        poseConfigs = poseConfigs != null ? new ArrayList<>(poseConfigs) : new ArrayList<>();
        stateStdDevs = stateStdDevs != null ? stateStdDevs : StdDevs.defaults();
        visionStdDevs = visionStdDevs != null ? visionStdDevs : StdDevs.defaults();
        visionMultiStdDevs = visionMultiStdDevs != null ? visionMultiStdDevs : StdDevs.defaults();
        autoPoseName = (autoPoseName == null || autoPoseName.isBlank()) ? "field" : autoPoseName;
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
            SlipStrategy slipStrategy,
            ImuStrategy imuStrategy,
            VisionStrategy visionStrategy,
            double slipThreshold,
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

        public enum SlipStrategy {
            OFF,
            SWERVE_VARIANCE
        }

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

        public static BackendConfig defualt() {
            return new BackendConfig(
                    SlipStrategy.OFF,
                    ImuStrategy.VIRTUAL_AXES,
                    VisionStrategy.DEFAULT,
                    0.2,
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

        public BackendConfig withSlipStrategy(SlipStrategy slipStrategy) {
            return new BackendConfig(
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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

        public BackendConfig withImuStrategy(ImuStrategy imuStrategy) {
            return new BackendConfig(
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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

        public BackendConfig withSlipThreshold(double slipThreshold) {
            return new BackendConfig(
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
                    slipStrategy,
                    imuStrategy,
                    visionStrategy,
                    slipThreshold,
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
            slipStrategy = slipStrategy != null ? slipStrategy : SlipStrategy.OFF;
            imuStrategy = imuStrategy != null ? imuStrategy : ImuStrategy.VIRTUAL_AXES;
            visionStrategy = visionStrategy != null ? visionStrategy : VisionStrategy.DEFAULT;
            if (!Double.isFinite(slipThreshold) || slipThreshold <= 0.0) {
                slipThreshold = 0.2;
            }
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
