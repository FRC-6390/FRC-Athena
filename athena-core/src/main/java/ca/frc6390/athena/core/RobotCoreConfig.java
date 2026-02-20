package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.DrivetrainSpeedConfigSupport;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrains.differential.sim.DifferentialSimulationConfig;
import ca.frc6390.athena.drivetrains.sim.CommonDrivetrainSimulationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
import ca.frc6390.athena.hardware.imu.AthenaImu;
import ca.frc6390.athena.logging.TelemetryRegistry;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.RegisterableMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanismFactory;
import ca.frc6390.athena.mechanisms.SuperstructureConfig;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;

/**
 * Declarative, Superstructure-style builder for {@link RobotCore.RobotCoreConfig}.
 *
 * <p>This is intended to "feel" like {@link SuperstructureConfig} and {@link MechanismConfig}:
 * create -> section lambdas -> build().</p>
 *
 * <p>The returned {@link RobotCore.RobotCoreConfig} can be passed to {@code super(...)} when
 * extending {@link RobotCore}.</p>
 */
public final class RobotCoreConfig {
    private RobotCoreConfig() {}

    public static BuilderStart create() {
        return new BuilderStart(null);
    }

    public static BuilderStart create(String name) {
        return new BuilderStart(name);
    }

    public static final class BuilderStart {
        private final String name;

        private BuilderStart(String name) {
            this.name = name;
        }

        public <T extends RobotDrivetrain<T>> Builder<T> drivetrain(
                Function<DrivetrainSection, RobotDrivetrainConfig<T>> selector) {
            Objects.requireNonNull(selector, "selector");
            RobotDrivetrainConfig<T> config = Objects.requireNonNull(selector.apply(new DrivetrainSection()),
                    "selector returned null drivetrain config");
            return new Builder<>(name, config);
        }
    }

    public static final class Builder<T extends RobotDrivetrain<T>> {
        private final String name;
        private final RobotDrivetrainConfig<T> drivetrainConfig;

        private RobotLocalizationConfig localizationConfig = RobotLocalizationConfig.defaults();
        private RobotVisionConfig visionConfig = RobotVisionConfig.defaults();
        private TelemetryRegistry.TelemetryConfig telemetryConfig = TelemetryRegistry.TelemetryConfig.defaults();
        private RobotCore.AutoConfig autoConfig = RobotCore.AutoConfig.defaults();

        private boolean autoInitResetEnabled = true;
        private boolean performanceMode = false;
        private boolean timingDebugEnabled = false;
        private boolean telemetryEnabled = true;
        private RobotCore.SystemConfig systemConfig = RobotCore.SystemConfig.defaults();

        private final List<RegisterableMechanism> mechanisms = new ArrayList<>();
        private RobotCoreHooks<T> hooks = RobotCoreHooks.<T>empty();

        private Builder(String name, RobotDrivetrainConfig<T> drivetrainConfig) {
            this.name = name;
            this.drivetrainConfig = Objects.requireNonNull(drivetrainConfig, "drivetrainConfig");
        }

        public Builder<T> localization(Consumer<LocalizationSection> section) {
            Objects.requireNonNull(section, "section");
            LocalizationSection l = new LocalizationSection(localizationConfig != null
                    ? localizationConfig
                    : RobotLocalizationConfig.defaults());
            section.accept(l);
            localizationConfig = l.config;
            return this;
        }

        public Builder<T> vision(Consumer<VisionSection> section) {
            Objects.requireNonNull(section, "section");
            VisionSection v = new VisionSection(visionConfig != null ? visionConfig : RobotVisionConfig.defaults());
            section.accept(v);
            visionConfig = v.config;
            return this;
        }

        public Builder<T> vision(ConfigurableCamera... cameras) {
            return vision(v -> v.cameras(cameras));
        }

        public Builder<T> telemetry(Consumer<TelemetrySection> section) {
            Objects.requireNonNull(section, "section");
            TelemetrySection t = new TelemetrySection(telemetryConfig != null
                    ? telemetryConfig
                    : TelemetryRegistry.TelemetryConfig.defaults());
            section.accept(t);
            telemetryConfig = t.config;
            return this;
        }

        /**
         * Configures global autonomous settings (planner PID, default pose key, and registry hooks).
         */
        public Builder<T> auto(Consumer<AutoSection> section) {
            Objects.requireNonNull(section, "section");
            AutoSection a = new AutoSection(autoConfig);
            section.accept(a);
            autoConfig = a.config;
            return this;
        }

        public Builder<T> mechanisms(Consumer<MechanismsSection> section) {
            Objects.requireNonNull(section, "section");
            MechanismsSection m = new MechanismsSection(mechanisms);
            section.accept(m);
            return this;
        }

        public Builder<T> core(Consumer<CoreSection> section) {
            Objects.requireNonNull(section, "section");
            CoreSection c = new CoreSection(this);
            section.accept(c);
            return this;
        }

        public Builder<T> hooks(Consumer<RobotCoreHooks.HooksSection<T>> section) {
            if (section != null) {
                hooks = hooks.hooks(section);
            }
            return this;
        }

        public Builder<T> inputs(Consumer<RobotCoreHooks.InputsSection<T>> section) {
            if (section != null) {
                hooks = hooks.inputs(section);
            }
            return this;
        }

        public RobotCore.RobotCoreConfig<T> build() {
            return new RobotCore.RobotCoreConfig<>(
                    drivetrainConfig,
                    localizationConfig != null ? localizationConfig : RobotLocalizationConfig.defaults(),
                    visionConfig != null ? visionConfig : RobotVisionConfig.defaults(),
                    autoInitResetEnabled,
                    telemetryConfig != null ? telemetryConfig : TelemetryRegistry.TelemetryConfig.defaults(),
                    List.copyOf(mechanisms),
                    performanceMode,
                    timingDebugEnabled,
                    telemetryEnabled,
                    autoConfig,
                    hooks,
                    systemConfig != null ? systemConfig : RobotCore.SystemConfig.defaults());
        }
    }

    public static final class DrivetrainSection {
        private DrivetrainSection() {}

        public RobotDrivetrainConfig<SwerveDrivetrain> swerve(SwerveDrivetrainConfig config) {
            return Objects.requireNonNull(config, "config");
        }

        public RobotDrivetrainConfig<SwerveDrivetrain> swerve(Consumer<SwerveSection> section) {
            Objects.requireNonNull(section, "section");
            SwerveSection s = new SwerveSection();
            section.accept(s);
            return s.config;
        }

        public RobotDrivetrainConfig<DifferentialDrivetrain> differential(DifferentialDrivetrainConfig config) {
            return Objects.requireNonNull(config, "config");
        }

        public RobotDrivetrainConfig<DifferentialDrivetrain> differential(Consumer<DifferentialSection> section) {
            Objects.requireNonNull(section, "section");
            DifferentialSection d = new DifferentialSection();
            section.accept(d);
            return d.config;
        }
    }

    public static final class SwerveSection {
        private final SwerveDrivetrainConfig config = new SwerveDrivetrainConfig();
        private SwerveSimulationConfig simulationConfig = SwerveSimulationConfig.defaults();

        private SwerveSection() {}

        public SwerveSection trackWidthMeters(double trackWidthMeters) {
            return moduleLocations(trackWidthMeters);
        }

        public SwerveSection trackWidthMeters(double trackWidthMeters, double wheelbaseMeters) {
            return moduleLocations(trackWidthMeters, wheelbaseMeters);
        }

        public SwerveSection moduleLocations(double trackWidthMeters) {
            config.hardware().moduleLocations(trackWidthMeters);
            return this;
        }

        public SwerveSection moduleLocations(double trackWidthMeters, double wheelbaseMeters) {
            config.hardware().moduleLocations(trackWidthMeters, wheelbaseMeters);
            return this;
        }

        public SwerveSection imu(AthenaImu imu, boolean inverted) {
            config.hardware().imu(imu, inverted);
            return this;
        }

        public SwerveSection ids(RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs ids) {
            config.hardware().ids(ids);
            return this;
        }

        public SwerveSection canbus(String canbus) {
            config.hardware().canbus(canbus);
            return this;
        }

        public SwerveSection modules(SwerveModuleConfig... modules) {
            config.hardware().modules(modules);
            return this;
        }

        public SwerveSection encoderOffsets(double... offsets) {
            config.hardware().encoderOffsets(offsets);
            return this;
        }

        public SwerveSection driveCurrentLimit(double amps) {
            config.hardware().driveCurrentLimit(amps);
            return this;
        }

        public SwerveSection steerCurrentLimit(double amps) {
            config.hardware().steerCurrentLimit(amps);
            return this;
        }

        public SwerveSection currents(double driveAmps, double steerAmps) {
            return driveCurrentLimit(driveAmps).steerCurrentLimit(steerAmps);
        }

        public SwerveSection fieldRelative(boolean enabled) {
            config.control().fieldRelative(enabled);
            return this;
        }

        public SwerveSection speedSource(Consumer<SpeedSourceSection> section) {
            Objects.requireNonNull(section, "section");
            DrivetrainSpeedConfigSupport speedSupport = new DrivetrainSpeedConfigSupport();
            section.accept(speedSourceSection(speedSupport));
            speedSupport.apply(config.speed());
            return this;
        }

        public SwerveSection simulation(Consumer<SwerveSimulationSection> section) {
            Objects.requireNonNull(section, "section");
            SwerveSimulationSection s = new SwerveSimulationSection(simulationConfig);
            section.accept(s);
            simulationConfig = s.config;
            config.simulation().config(simulationConfig);
            return this;
        }
    }

    private abstract static class CommonSimulationSection<
            C extends CommonDrivetrainSimulationConfig<C>,
            Self extends CommonSimulationSection<C, Self>> {
        protected C config;

        protected CommonSimulationSection(C config) {
            this.config = Objects.requireNonNull(config, "config");
        }

        protected abstract Self self();

        public final Self robotMassKg(double kg) {
            config = config.withRobotMassKg(kg);
            return self();
        }

        public final Self nominalVoltage(double volts) {
            config = config.withNominalVoltage(volts);
            return self();
        }

        public final Self robotMomentOfInertia(double moi) {
            config = config.withRobotMomentOfInertia(moi);
            return self();
        }
    }

    public static final class SwerveSimulationSection
            extends CommonSimulationSection<SwerveSimulationConfig, SwerveSimulationSection> {

        private SwerveSimulationSection(SwerveSimulationConfig config) {
            super(config);
        }

        @Override
        protected SwerveSimulationSection self() {
            return this;
        }

        public SwerveSimulationSection wheelCoefficientOfFriction(double coef) {
            config = config.withWheelCoefficientOfFriction(coef);
            return this;
        }

        public SwerveSimulationSection maxSpeedScale(double scale) {
            config = config.withMaxSpeedScale(scale);
            return this;
        }
    }

    public static final class DifferentialSection {
        private final DifferentialDrivetrainConfig config = new DifferentialDrivetrainConfig();
        private DifferentialSimulationConfig simulationConfig = DifferentialSimulationConfig.defaults();

        private DifferentialSection() {}

        public DifferentialSection imu(AthenaImu imu, boolean inverted) {
            config.hardware().imu(imu, inverted);
            return this;
        }

        public DifferentialSection trackWidthMeters(double meters) {
            config.hardware().trackWidth(meters);
            return this;
        }

        public DifferentialSection canbus(String canbus) {
            config.hardware().canbus(canbus);
            return this;
        }

        public DifferentialSection currentLimit(double amps) {
            config.hardware().currentLimit(amps);
            return this;
        }

        public DifferentialSection speedSource(Consumer<SpeedSourceSection> section) {
            Objects.requireNonNull(section, "section");
            DrivetrainSpeedConfigSupport speedSupport = new DrivetrainSpeedConfigSupport();
            section.accept(speedSourceSection(speedSupport));
            speedSupport.apply(config.speed());
            return this;
        }

        public DifferentialSection simulation(Consumer<DifferentialSimulationSection> section) {
            Objects.requireNonNull(section, "section");
            DifferentialSimulationSection s = new DifferentialSimulationSection(simulationConfig);
            section.accept(s);
            simulationConfig = s.config;
            config.simulation().config(simulationConfig);
            return this;
        }
    }

    public static final class DifferentialSimulationSection
            extends CommonSimulationSection<DifferentialSimulationConfig, DifferentialSimulationSection> {

        private DifferentialSimulationSection(DifferentialSimulationConfig config) {
            super(config);
        }

        @Override
        protected DifferentialSimulationSection self() {
            return this;
        }

        public DifferentialSimulationSection gearRatio(double ratio) {
            config = config.withGearRatio(ratio);
            return this;
        }

        public DifferentialSimulationSection wheelDiameterMeters(double meters) {
            config = config.withWheelDiameterMeters(meters);
            return this;
        }

        public DifferentialSimulationSection trackWidthMeters(double meters) {
            config = config.withTrackWidthMeters(meters);
            return this;
        }

        public DifferentialSimulationSection motorsPerSide(int motors) {
            config = config.withMotorsPerSide(motors);
            return this;
        }

    }

    @FunctionalInterface
    private interface SpeedSourceConfigurer {
        void add(String name, boolean enabledByDefault);

        default void blend(
                String target,
                String source,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {}

        default void blend(
                String target,
                String left,
                String right,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {}

        default void blendToOutput(
                String source,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {}
    }

    private static SpeedSourceSection speedSourceSection(DrivetrainSpeedConfigSupport speedSupport) {
        return new SpeedSourceSection(new SpeedSourceConfigurer() {
            @Override
            public void add(String name, boolean enabledByDefault) {
                speedSupport.source(name, enabledByDefault);
            }

            @Override
            public void blend(
                    String target,
                    String source,
                    RobotSpeeds.BlendMode blendMode,
                    RobotSpeeds.SpeedAxis... axes) {
                speedSupport.blend(target, source, blendMode, axes);
            }

            @Override
            public void blend(
                    String target,
                    String left,
                    String right,
                    RobotSpeeds.BlendMode blendMode,
                    RobotSpeeds.SpeedAxis... axes) {
                speedSupport.blend(target, left, right, blendMode, axes);
            }

            @Override
            public void blendToOutput(
                    String source,
                    RobotSpeeds.BlendMode blendMode,
                    RobotSpeeds.SpeedAxis... axes) {
                speedSupport.outputBlend(source, blendMode, axes);
            }
        });
    }

    public static final class SpeedSourceSection {
        private final SpeedSourceConfigurer configurer;

        private SpeedSourceSection(SpeedSourceConfigurer configurer) {
            this.configurer = Objects.requireNonNull(configurer, "configurer");
        }

        public SpeedSourceSection add(String name, boolean enabledByDefault) {
            configurer.add(name, enabledByDefault);
            return this;
        }

        public SpeedSourceSection add(String name) {
            return add(name, true);
        }

        public SpeedSourceSection blend(
                String target,
                String source,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {
            configurer.blend(target, source, blendMode, axes);
            return this;
        }

        public SpeedSourceSection blend(
                String target,
                String left,
                String right,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {
            configurer.blend(target, left, right, blendMode, axes);
            return this;
        }

        public SpeedSourceSection blendToOutput(
                String source,
                RobotSpeeds.BlendMode blendMode,
                RobotSpeeds.SpeedAxis... axes) {
            configurer.blendToOutput(source, blendMode, axes);
            return this;
        }
    }

    public static final class LocalizationSection {
        private RobotLocalizationConfig config;

        private LocalizationSection(RobotLocalizationConfig config) {
            this.config = Objects.requireNonNull(config, "config");
        }

        public LocalizationSection config(RobotLocalizationConfig config) {
            this.config = Objects.requireNonNull(config, "config");
            return this;
        }

        public LocalizationSection vision(double vXStd, double vYStd, double vThetaStd) {
            config = RobotLocalizationConfig.vision(vXStd, vYStd, vThetaStd);
            return this;
        }

        public LocalizationSection visionStdDevs(double vXStd, double vYStd, double vThetaStd) {
            config.estimation().vision(vXStd, vYStd, vThetaStd);
            return this;
        }

        public LocalizationSection visionEnabled(boolean enabled) {
            config.estimation().visionEnabled(enabled);
            return this;
        }

        public LocalizationSection use3d() {
            config.estimation().use3d();
            return this;
        }

        public LocalizationSection use2d() {
            config.estimation().use2d();
            return this;
        }

        public LocalizationSection poseConfig(ca.frc6390.athena.core.localization.PoseConfig poseConfig) {
            config.poses().pose(poseConfig);
            return this;
        }

        public LocalizationSection boundingBox(
                String name,
                ca.frc6390.athena.core.localization.PoseBoundingBox2d box) {
            config.poses().boundingBox(name, box);
            return this;
        }

        public LocalizationSection boundingBox(
                String name,
                edu.wpi.first.math.geometry.Translation2d cornerA,
                edu.wpi.first.math.geometry.Translation2d cornerB) {
            config.poses().boundingBox(name, cornerA, cornerB);
            return this;
        }

    }

    /**
     * Builder facade for {@link RobotCore.AutoConfig}.
     *
     * <p>This section configures global auto behavior used by {@link RobotAuto}, while
     * {@link RobotAuto.AutoBuildCtx} configures per-build command composition.</p>
     */
    public static final class AutoSection {
        private RobotCore.AutoConfig config;

        private AutoSection(RobotCore.AutoConfig config) {
            this.config = config != null ? config : RobotCore.AutoConfig.defaults();
        }

        /**
         * Replaces the current auto config object.
         */
        public AutoSection config(RobotCore.AutoConfig config) {
            this.config = config != null ? config : RobotCore.AutoConfig.defaults();
            return this;
        }

        /**
         * Sets translation/rotation planner PID gains.
         */
        public AutoSection pid(
                double tP,
                double tI,
                double tD,
                double rP,
                double rI,
                double rD) {
            config = config.pid(tP, tI, tD, rP, rI, rD);
            return this;
        }

        /**
         * Configures planner PID values through the localization auto-planner section API.
         */
        public AutoSection pid(Consumer<RobotLocalizationConfig.AutoPlannerPidSection> section) {
            config = config.pid(section);
            return this;
        }

        /**
         * Sets the pose key used by auto planner reset behavior.
         */
        public AutoSection pose(String poseName) {
            config = config.pose(poseName);
            return this;
        }

        /**
         * Adds auto registry bindings that run against {@link RobotAuto.RegistrySection}.
         */
        public AutoSection registry(Consumer<RobotAuto.RegistrySection> section) {
            config = config.registry(section);
            return this;
        }
    }

    public static final class VisionSection {
        private RobotVisionConfig config;

        private VisionSection(RobotVisionConfig config) {
            this.config = Objects.requireNonNull(config, "config");
        }

        public VisionSection config(RobotVisionConfig config) {
            this.config = Objects.requireNonNull(config, "config");
            return this;
        }

        public VisionSection camera(ConfigurableCamera camera) {
            if (camera == null) {
                return this;
            }
            config.addCamera(camera);
            return this;
        }

        public VisionSection cameras(ConfigurableCamera... cameras) {
            if (cameras == null || cameras.length == 0) {
                return this;
            }
            config.addCameras(cameras);
            return this;
        }
    }

    public static final class TelemetrySection {
        private TelemetryRegistry.TelemetryConfig config;

        private TelemetrySection(TelemetryRegistry.TelemetryConfig config) {
            this.config = Objects.requireNonNull(config, "config");
        }

        public TelemetrySection config(TelemetryRegistry.TelemetryConfig config) {
            this.config = Objects.requireNonNull(config, "config");
            return this;
        }

        public TelemetrySection defaultPeriodMs(int ms) {
            config = config.defaultPeriodMs(ms);
            return this;
        }

        public TelemetrySection diskEnabled(boolean enabled) {
            config = config.diskEnabled(enabled);
            return this;
        }

        public TelemetrySection networkTablesEnabled(boolean enabled) {
            config = config.networkTablesEnabled(enabled);
            return this;
        }

        public TelemetrySection prefix(String prefix) {
            config = config.diskPrefix(prefix);
            return this;
        }

        public TelemetrySection networkTablePrefix(String prefix) {
            config = config.networkTablePrefix(prefix);
            return this;
        }

        public TelemetrySection dataLogDirectory(String directory) {
            config = config.dataLogDirectory(directory);
            return this;
        }

        public TelemetrySection dataLogRetentionCount(int count) {
            config = config.dataLogRetentionCount(count);
            return this;
        }
    }

    public static final class MechanismsSection {
        private final List<RegisterableMechanism> out;

        private MechanismsSection(List<RegisterableMechanism> out) {
            this.out = Objects.requireNonNull(out, "out");
        }

        public MechanismsSection existing(RegisterableMechanism entry) {
            if (entry != null) {
                out.add(entry);
            }
            return this;
        }

        public MechanismsSection mechanism(Mechanism mechanism) {
            return existing(mechanism);
        }

        public MechanismsSection superstructure(SuperstructureMechanism<?, ?> superstructure) {
            return existing(superstructure);
        }

        public MechanismsSection mechanism(MechanismConfig<? extends Mechanism> config) {
            if (config != null) {
                out.add(new LazyRegisterable(() -> config.build()));
            }
            return this;
        }

        public MechanismsSection superstructure(SuperstructureConfig<?, ?> config) {
            if (config != null) {
                out.add(new LazyRegisterable(config::build));
            }
            return this;
        }
    }

    public static final class CoreSection {
        private final Builder<?> owner;

        private CoreSection(Builder<?> owner) {
            this.owner = Objects.requireNonNull(owner, "owner");
        }

        public CoreSection autoInitResetEnabled(boolean enabled) {
            owner.autoInitResetEnabled = enabled;
            return this;
        }

        public CoreSection performanceMode(boolean enabled) {
            owner.performanceMode = enabled;
            return this;
        }

        public CoreSection timingDebugEnabled(boolean enabled) {
            owner.timingDebugEnabled = enabled;
            return this;
        }

        public CoreSection telemetryEnabled(boolean enabled) {
            owner.telemetryEnabled = enabled;
            return this;
        }

        public CoreSection system(Consumer<SystemSection> section) {
            Objects.requireNonNull(section, "section");
            SystemSection s = new SystemSection(owner);
            section.accept(s);
            return this;
        }
    }

    public static final class SystemSection {
        private final Builder<?> owner;

        private SystemSection(Builder<?> owner) {
            this.owner = Objects.requireNonNull(owner, "owner");
            if (this.owner.systemConfig == null) {
                this.owner.systemConfig = RobotCore.SystemConfig.defaults();
            }
        }

        /**
         * Sets Linux {@code vm.overcommit_memory}.
         * Typical values: {@code 0}=heuristic, {@code 1}=always overcommit, {@code 2}=strict.
         */
        public SystemSection vmOvercommitMode(int mode) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    mode,
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Sets Linux {@code vm.overcommit_ratio} (percent).
         * Primarily used when overcommit mode is {@code 2}.
         */
        public SystemSection vmOvercommitRatio(int ratio) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    ratio,
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Sets Linux {@code vm.swappiness}.
         * Higher values make the kernel more willing to swap memory pages.
         */
        public SystemSection vmSwappiness(int value) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    value,
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Configures Athena-managed loopback swap.
         *
         * @param enabled whether Athena should enable loopback swap
         * @param sizeMiB swap file size in MiB when enabled
         */
        public SystemSection loopSwap(boolean enabled, int sizeMiB) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    enabled,
                    sizeMiB,
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Controls NI {@code SystemWebServer} at startup.
         */
        public SystemSection systemWebServerEnabled(boolean enabled) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    enabled,
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Enables/disables Athena's config/diagnostics HTTP server.
         * This is a runtime service toggle, not an OS kernel tweak.
         */
        public SystemSection configServerEnabled(boolean enabled) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    enabled,
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Enables/disables Athena telemetry publishing.
         * This is a runtime service toggle, not an OS kernel tweak.
         */
        public SystemSection telemetryEnabled(boolean enabled) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    enabled,
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Enables/disables NetworkTables datalogging.
         * This is a runtime logging toggle, not an OS kernel tweak.
         */
        public SystemSection networkTablesDataLogEnabled(boolean enabled) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    current.tweaksEnabled(),
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    enabled);
            return this;
        }

        /**
         * Master toggle for OS-level tweaks (sysctl, loop swap, SystemWebServer commands).
         * When disabled, tweak calls are retained in config but are no-ops at runtime.
         */
        public SystemSection tweaksEnabled(boolean enabled) {
            RobotCore.SystemConfig current = owner.systemConfig;
            owner.systemConfig = new RobotCore.SystemConfig(
                    enabled,
                    current.vmOvercommitMode(),
                    current.vmOvercommitRatio(),
                    current.vmSwappiness(),
                    current.loopSwapEnabled(),
                    current.loopSwapSizeMiB(),
                    current.systemWebServerEnabled(),
                    current.configServerEnabled(),
                    current.telemetryEnabled(),
                    current.networkTablesDataLogEnabled());
            return this;
        }

        /**
         * Applies conservative roboRIO defaults for this system section.
         */
        public SystemSection rioDefaults() {
            owner.systemConfig = RobotCore.SystemConfig.rioDefaults();
            return this;
        }
    }

    private static final class LazyRegisterable implements RegisterableMechanismFactory {
        private final java.util.function.Supplier<? extends RegisterableMechanism> supplier;
        private volatile RegisterableMechanism built;

        private LazyRegisterable(java.util.function.Supplier<? extends RegisterableMechanism> supplier) {
            this.supplier = Objects.requireNonNull(supplier, "supplier");
        }

        @Override
        public RegisterableMechanism build() {
            RegisterableMechanism cached = built;
            if (cached != null) {
                return cached;
            }
            synchronized (this) {
                if (built == null) {
                    built = supplier.get();
                }
                return built;
            }
        }

        @Override
        public List<Mechanism> flattenForRegistration() {
            RegisterableMechanism entry = build();
            return entry != null ? entry.flattenForRegistration() : List.of();
        }
    }
}
