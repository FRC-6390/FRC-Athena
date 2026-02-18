package ca.frc6390.athena.drivetrains.swerve;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotTime;
import ca.frc6390.athena.commands.control.SwerveDriveCommand;
import ca.frc6390.athena.controllers.SimpleMotorFeedForwardsSendable;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.localization.PoseEstimatorFactory;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.localization.RobotDrivetrainLocalizationFactory;
import ca.frc6390.athena.odometry.SlipCompensatingSwerveDrivePoseEstimator;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveDrivetrainSimulation;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase
    implements RobotDrivetrain<SwerveDrivetrain>, RobotDrivetrainLocalizationFactory {

  private static final double DRIFT_TURN_DEADBAND_RAD_PER_SEC = 0.05;
  private static final double MODULE_HEADING_EPSILON = 1e-6;
  private static final double MODULE_ANGLE_UPDATE_EPSILON_RAD = 1e-4;
  private static final Pose2d ZERO_POSE = new Pose2d();

  public SwerveModule[] swerveModules;
  public SwerveDriveKinematics kinematics;
  public PIDController driftpid;
  public boolean enableDriftCorrection, fieldRelative;
  public double desiredHeading, driftActivationSpeed;
  public Imu imu;
  public RobotSpeeds robotSpeeds;
  private final MotionLimits motionLimits;
  private final ControlRuntimeSection controlSection = new ControlRuntimeSection();
  private final SpeedsRuntimeSection speedsSection = new SpeedsRuntimeSection();
  private final ModulesRuntimeSection modulesSection = new ModulesRuntimeSection();
  private final HardwareRuntimeSection hardwareSection = new HardwareRuntimeSection();
  private final SysIdRuntimeSection sysIdSection = new SysIdRuntimeSection();
  private final ImuRuntimeSection imuSection = new ImuRuntimeSection();
  private final SimulationRuntimeSection simulationSection = new SimulationRuntimeSection();
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Drivetrain/SwerveStates", SwerveModuleState.struct).publish();
  private static final double SWERVE_STATE_PUBLISH_PERIOD_SECONDS = 0.1;
  private double lastSwerveStatePublishSeconds = Double.NaN;
  private SwerveDrivetrainSimulation simulation;
  private Field2d simulationField;
  private double lastSimulationTimestamp = -1;
  private final ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();
  private final ChassisSpeeds calculatedSpeeds = new ChassisSpeeds();
  private final ChassisSpeeds lastLimitedSpeeds = new ChassisSpeeds();
  private final ChassisSpeeds velocityLimitedSpeeds = new ChassisSpeeds();
  private final ChassisSpeeds accelerationLimitedSpeeds = new ChassisSpeeds();
  private final SwerveModuleState[] desiredModuleStates;
  private final Translation2d[] moduleLocations;
  private final double maxDriveVelocityMetersPerSecond;
  private double lastMotionLimitTimestampSeconds = Double.NaN;
  private SimpleMotorFeedForwardsSendable driveFeedforward;
  private boolean driveFeedforwardEnabled = false;
  private double nominalVoltage = 12.0;
  private SysIdRoutine sysIdRoutine;
  private double sysIdRampRateVoltsPerSecond = 1.0;
  private double sysIdStepVoltage = 7.0;
  private double sysIdTimeoutSeconds = 10.0;
  private double sysIdLastVoltage = 0.0;
  private boolean sysIdActive = false;
  private final Rotation2d sysIdSteerAngle = new Rotation2d();
  private String ntRootPath;
  private RobotNetworkTables.Node ntModulesNode;
  private RobotNetworkTables.Node ntDriftNode;
  private RobotNetworkTables.Node ntSysIdNode;
  private RobotNetworkTables.Node ntSimNode;
  private RobotNetworkTables.Node[] ntModuleNodes;
  
  public SwerveDrivetrain(Imu imu, SwerveModuleConfig... modules) {

    this.imu = imu;

    enableDriftCorrection = false; 
    driftpid = new PIDController(0, 0,0);
    driftpid.enableContinuousInput(-Math.PI, Math.PI);

    swerveModules = new SwerveModule[modules.length];
    desiredModuleStates = new SwerveModuleState[modules.length];
    moduleLocations = new Translation2d[modules.length];
    double maxVelocity = 0;
    double minVelocity = Double.POSITIVE_INFINITY;
    double maxModuleRadius = 0;
    for (int i = 0; i < modules.length; i++) {
      swerveModules[i] = new SwerveModule(modules[i]);
      desiredModuleStates[i] = new SwerveModuleState(0.0, new Rotation2d());
      double moduleMax = modules[i].maxSpeedMetersPerSecond();
      maxVelocity = Math.max(maxVelocity, moduleMax);
      minVelocity = Math.min(minVelocity, moduleMax);
      moduleLocations[i] = swerveModules[i].getModuleLocation();
      maxModuleRadius = Math.max(maxModuleRadius, moduleLocations[i].getNorm());
    }

    if (minVelocity < Double.POSITIVE_INFINITY && maxVelocity > 1e-6
        && (maxVelocity - minVelocity) / maxVelocity > 0.05) {
      DriverStation.reportWarning(
          "Swerve modules report different max speeds; limiting drivetrain max speed to the slowest module.",
          false);
      maxVelocity = minVelocity;
    }

    maxDriveVelocityMetersPerSecond = maxVelocity;
    kinematics = new SwerveDriveKinematics(moduleLocations);
    double maxAngularVelocity =
        maxModuleRadius > 1e-6 ? maxDriveVelocityMetersPerSecond / maxModuleRadius : maxDriveVelocityMetersPerSecond;
    robotSpeeds = new RobotSpeeds(maxDriveVelocityMetersPerSecond, maxAngularVelocity);
    motionLimits = new MotionLimits()
        .setBaseDriveLimits(MotionLimits.DriveLimits.fromRobotSpeeds(robotSpeeds));
    imu.addVirtualAxis("drift", imu::getYaw);
    imu.setVirtualAxis("drift", imu.getVirtualAxis("driver"));
    desiredHeading = imu.getVirtualAxis("drift").getRadians();
    setNominalVoltage(nominalVoltage);
    
  }

  @Override
  public SwerveDrivetrain control(Consumer<RobotDrivetrain.ControlSection> section) {
    if (section != null) {
      section.accept(controlSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.ControlSection control() {
    return controlSection;
  }

  @Override
  public SwerveDrivetrain speeds(Consumer<RobotDrivetrain.SpeedsSection> section) {
    if (section != null) {
      section.accept(speedsSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.SpeedsSection speeds() {
    return speedsSection;
  }

  @Override
  public SwerveDrivetrain modules(Consumer<RobotDrivetrain.ModulesSection> section) {
    if (section != null) {
      section.accept(modulesSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.ModulesSection modules() {
    return modulesSection;
  }

  @Override
  public SwerveDrivetrain hardware(Consumer<RobotDrivetrain.HardwareSection> section) {
    if (section != null) {
      section.accept(hardwareSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.HardwareSection hardware() {
    return hardwareSection;
  }

  @Override
  public SwerveDrivetrain sysId(Consumer<RobotDrivetrain.SysIdSection> section) {
    if (section != null) {
      section.accept(sysIdSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.SysIdSection sysId() {
    return sysIdSection;
  }

  @Override
  public SwerveDrivetrain imu(Consumer<RobotDrivetrain.ImuSection> section) {
    if (section != null) {
      section.accept(imuSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.ImuSection imu() {
    return imuSection;
  }

  @Override
  public SwerveDrivetrain simulation(Consumer<RobotDrivetrain.SimulationSection> section) {
    if (section != null) {
      section.accept(simulationSection);
    }
    return this;
  }

  @Override
  public RobotDrivetrain.SimulationSection simulation() {
    return simulationSection;
  }

  @Override
  public RobotSpeeds robotSpeeds() {
    return speedsModel();
  }

  public SwerveDrivetrain withDriftCorretion(PIDController controller){
    driftpid = controller;
    driftpid.enableContinuousInput(-Math.PI, Math.PI);
    enableDriftCorrection = true;
    return this;
  }
  
  public SwerveDriveKinematics kinematicsModel() {
    return kinematics;
  } 

  private SwerveModulePosition[] positions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      SwerveModulePosition modulePosition = swerveModules[i].getPostion();
      positions[i] = new SwerveModulePosition(
          modulePosition.distanceMeters,
          modulePosition.angle);
    }
    return positions;
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  private void calculateModuleStates(ChassisSpeeds speed, SwerveModuleState[] states) {
    double vx = speed.vxMetersPerSecond;
    double vy = speed.vyMetersPerSecond;
    double omega = speed.omegaRadiansPerSecond;
    boolean stationary = vx == 0.0 && vy == 0.0 && omega == 0.0;

    SwerveModuleState[] calculatedStates = stationary ? null : kinematics.toSwerveModuleStates(speed);

    for (int i = 0; i < states.length; i++) {
      SwerveModuleState state = states[i];
      if (stationary) {
        state.speedMetersPerSecond = 0.0;
        continue;
      }

      SwerveModuleState calculated = calculatedStates[i];
      state.speedMetersPerSecond = calculated.speedMetersPerSecond;
      state.angle = calculated.angle;
    }
  }


  private double driftCorrection(ChassisSpeeds speeds) {
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double commandedOmega = Math.abs(speeds.omegaRadiansPerSecond);
    double measuredOmega = Math.abs(imu.getVelocityZ().getRadians());

    // Hold heading only while translating straight; never fight intentional turns.
    double driftHeadingRadians = imu.getVirtualAxis("drift").getRadians();
    if (commandedOmega > DRIFT_TURN_DEADBAND_RAD_PER_SEC
        || measuredOmega > DRIFT_TURN_DEADBAND_RAD_PER_SEC
        || linearSpeed < Math.max(0.0, driftActivationSpeed)) {
      desiredHeading = driftHeadingRadians;
      return 0;
    } else {
      return driftpid.calculate(driftHeadingRadians, desiredHeading);
    }
  }

  public SwerveDrivetrain driftCorrectionPid(PIDController controller) {
    driftpid = controller;
    return this;
  }
  
  public void driftCorrectionEnabled(boolean enabled) {
    enableDriftCorrection = enabled;
  }

  public boolean driftCorrectionEnabled() {
    return enableDriftCorrection;
  }

  public void fieldRelative(boolean fieldRelative) {
      this.fieldRelative = fieldRelative;
  }

  public SwerveDrivetrain driveFeedforward(SimpleMotorFeedforward feedforward) {
    if (feedforward == null) {
      driveFeedforward = null;
      driveFeedforwardEnabled(false);
      for (SwerveModule module : swerveModules) {
        module.setDriveFeedforward(null);
      }
      return this;
    }
    driveFeedforward = new SimpleMotorFeedForwardsSendable(
        feedforward.getKs(),
        feedforward.getKv(),
        feedforward.getKa());
    for (SwerveModule module : swerveModules) {
      module.setDriveFeedforward(driveFeedforward);
    }
    driveFeedforwardEnabled(true);
    return this;
  }

  public void driveFeedforwardEnabled(boolean enabled) {
    driveFeedforwardEnabled = driveFeedforward != null && enabled;
    for (SwerveModule module : swerveModules) {
      module.setDriveFeedforwardEnabled(driveFeedforwardEnabled);
    }
  }

  public boolean driveFeedforwardEnabled() {
    return driveFeedforwardEnabled;
  }

  public boolean driveFeedforwardConfigured() {
    return driveFeedforward != null;
  }

  private double sysIdRampRateVoltsPerSecond() {
    return sysIdRampRateVoltsPerSecond;
  }

  private void sysIdRampRateVoltsPerSecond(double rampRate) {
    if (!Double.isFinite(rampRate) || rampRate <= 0.0) {
      return;
    }
    sysIdRampRateVoltsPerSecond = rampRate;
    invalidateSysIdRoutine();
  }

  private double sysIdStepVoltage() {
    return sysIdStepVoltage;
  }

  private void sysIdStepVoltage(double stepVoltage) {
    if (!Double.isFinite(stepVoltage) || stepVoltage <= 0.0) {
      return;
    }
    sysIdStepVoltage = stepVoltage;
    invalidateSysIdRoutine();
  }

  private double sysIdTimeoutSeconds() {
    return sysIdTimeoutSeconds;
  }

  private void sysIdTimeoutSeconds(double timeoutSeconds) {
    if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0) {
      return;
    }
    sysIdTimeoutSeconds = timeoutSeconds;
    invalidateSysIdRoutine();
  }

  private boolean sysIdActive() {
    return sysIdActive;
  }

  private Imu imuDevice() {
      return imu;
  }

  private void applyNeutralMode(MotorNeutralMode mode) {
    for (SwerveModule module : swerveModules) {
      module.setNeutralMode(mode);
    }
  }

  private RobotSpeeds speedsModel() {
    return robotSpeeds;
  }

  private MotionLimits limitsModel() {
    return motionLimits;
  }

  private void resetControlState() {
    robotSpeeds.stop();
    setChassisSpeeds(lastCommandedSpeeds, 0.0, 0.0, 0.0);
    setChassisSpeeds(lastLimitedSpeeds, 0.0, 0.0, 0.0);
    lastMotionLimitTimestampSeconds = Double.NaN;
    desiredHeading = imu.getVirtualAxis("drift").getRadians();
    for (SwerveModule module : swerveModules) {
      module.stop();
    }
  }

  @Override
  public void update() {
    imu.update();
    
    if (sysIdActive) {
      for (SwerveModule module : swerveModules) {
        module.refresh();
      }
      return;
    }

    robotSpeeds.calculate(calculatedSpeeds);
    ChassisSpeeds speed = calculatedSpeeds;
    double nowSeconds = nowSeconds();

    if (enableDriftCorrection) {
      speed.omegaRadiansPerSecond += driftCorrection(speed);
    }
    speed = applyMotionLimits(speed, nowSeconds);

    calculateModuleStates(speed, desiredModuleStates);

    publishSwerveStatesIfDue(desiredModuleStates, nowSeconds);

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxDriveVelocityMetersPerSecond);

    setModuleStates(desiredModuleStates);   

    setChassisSpeeds(
        lastCommandedSpeeds,
        speed.vxMetersPerSecond,
        speed.vyMetersPerSecond,
        speed.omegaRadiansPerSecond);
  }

  public SwerveDrivetrain configureSimulation(SwerveSimulationConfig config) {
    if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return this;
    }
    simulation = new SwerveDrivetrainSimulation(this, config);
    if (config != null) {
      setNominalVoltage(config.getNominalVoltage());
    }
    if (simulationField == null) {
      simulationField = new Field2d();
    }
    simulationField.setRobotPose(ZERO_POSE);
    lastSimulationTimestamp = -1;
    return this;
  }

  private boolean simulationEnabled() {
    return simulation != null;
  }

  private Pose2d simulationPose() {
    return simulation != null ? simulation.getPose() : ZERO_POSE;
  }

  private void simulationPose(Pose2d pose) {
    if (simulation != null) {
      simulation.resetPose(pose);
    }
  }

  private Command driveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    return new SwerveDriveCommand(this, xInput, yInput, thetaInput, fieldRelative);
  }

  private void ensureNetworkTableNodes(RobotNetworkTables.Node node) {
    String path = node.path();
    if (path != null && path.equals(ntRootPath) && ntModulesNode != null && ntModuleNodes != null) {
      return;
    }
    ntRootPath = path;
    ntModulesNode = node.child("SwerveModules");
    ntDriftNode = node.child("DriftCorrection");
    ntSysIdNode = node.child("SysId");
    ntSimNode = node.child("Sim");
    ntModuleNodes = new RobotNetworkTables.Node[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      ntModuleNodes[i] = ntModulesNode.child("Module" + i);
    }
  }

  @Override
  public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
    if (node == null) {
      return node;
    }
    if (!node.robot().isPublishingEnabled()) {
      return node;
    }

    RobotDrivetrain.super.networkTables(node);

    ensureNetworkTableNodes(node);

    for (int i = 0; i < swerveModules.length; i++) {
      SwerveModule module = swerveModules[i];
      if (module != null && ntModuleNodes != null && i < ntModuleNodes.length) {
        module.networkTables(ntModuleNodes[i]);
      }
    }

    ntDriftNode.putBoolean("enabled", driftCorrectionEnabled());
    ntDriftNode.putDouble("desiredHeadingDeg", desiredHeading);

    ntSysIdNode.putDouble("rampRateVPerSec", sysIdRampRateVoltsPerSecond());
    ntSysIdNode.putDouble("stepVoltageV", sysIdStepVoltage());
    ntSysIdNode.putDouble("timeoutSec", sysIdTimeoutSeconds());
    ntSysIdNode.putBoolean("active", sysIdActive());

    if (simulation != null) {
      Pose2d pose = simulationPose();
      ntSimNode.putDouble("xM", pose.getX());
      ntSimNode.putDouble("yM", pose.getY());
      ntSimNode.putDouble("rotDeg", pose.getRotation().getDegrees());
    }

    return node;
  }

  @Override
  public void periodic() {
      update();
  }

  @Override
  public void simulationPeriodic() {
      if (simulation != null) {
        double now = nowSeconds();
        if (lastSimulationTimestamp < 0) {
          lastSimulationTimestamp = now;
        }
        double dt = now - lastSimulationTimestamp;
        lastSimulationTimestamp = now;
        simulation.update(dt > 0 ? dt : 0.02, lastCommandedSpeeds);
        if (simulationField != null) {
          simulationField.setRobotPose(simulation.getPose());
        }
      }
  }

  @Override
  public RobotLocalization<SwerveModulePosition[]> createLocalization(RobotLocalizationConfig config) {
    RobotLocalizationConfig effectiveConfig = config != null ? config : RobotLocalizationConfig.defaults();
    RobotLocalizationConfig.BackendConfig backend = effectiveConfig.backend();
    PoseEstimatorFactory<SwerveModulePosition[]> factory = new PoseEstimatorFactory<>() {
      @Override
      public SwerveDrivePoseEstimator create2d(
              Pose2d startPose,
              Matrix<N3, N1> stateStdDevs,
              Matrix<N3, N1> visionStdDevs) {
        if (backend != null
                && backend.slipStrategy() == RobotLocalizationConfig.BackendConfig.SlipStrategy.SWERVE_VARIANCE) {
          SlipCompensatingSwerveDrivePoseEstimator estimator =
                  new SlipCompensatingSwerveDrivePoseEstimator(
                          kinematics,
                          imu.getYaw(),
                          positions(),
                          startPose,
                          stateStdDevs,
                          visionStdDevs);
          estimator.setSlipThreshold(backend.slipThreshold());
          return estimator;
        }
        return new SwerveDrivePoseEstimator(
                kinematics,
                imu.getYaw(),
                positions(),
                startPose,
                stateStdDevs,
                visionStdDevs);
      }

      @Override
      public SwerveDrivePoseEstimator3d create3d(
              Pose3d startPose,
              Matrix<N4, N1> stateStdDevs,
              Matrix<N4, N1> visionStdDevs) {
        Rotation3d gyroRotation = getImuRotation3d();
        return new SwerveDrivePoseEstimator3d(
                kinematics,
                gyroRotation,
                positions(),
                startPose,
                stateStdDevs,
                visionStdDevs);
      }
    };

    return new RobotLocalization<>(factory, effectiveConfig, robotSpeeds, imu, this::positions);
  }

  private Rotation3d getImuRotation3d() {
    return new Rotation3d(
            imu.getRoll().getRadians(),
            imu.getPitch().getRadians(),
            imu.getYaw().getRadians());
  }

  private SysIdRoutine getSysIdRoutine() {
    if (sysIdRoutine == null) {
      sysIdRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(
              edu.wpi.first.units.Units.Volts.of(sysIdRampRateVoltsPerSecond).per(edu.wpi.first.units.Units.Second),
              edu.wpi.first.units.Units.Volts.of(sysIdStepVoltage),
              edu.wpi.first.units.Units.Seconds.of(sysIdTimeoutSeconds)),
          new SysIdRoutine.Mechanism(this::applySysIdVoltage, this::logSysIdData, this, getName()));
    }
    return sysIdRoutine;
  }

  private void invalidateSysIdRoutine() {
    sysIdRoutine = null;
  }

  private void applySysIdVoltage(Voltage voltage) {
    double requested = voltage.in(edu.wpi.first.units.Units.Volts);
    double voltageLimit = getVoltageLimit();
    double applied = MathUtil.clamp(requested, -voltageLimit, voltageLimit);
    sysIdLastVoltage = applied;
    for (SwerveModule module : swerveModules) {
      module.setSteerAngle(sysIdSteerAngle);
      module.setDriveVoltage(applied);
    }
  }

  private void logSysIdData(SysIdRoutineLog log) {
    for (int i = 0; i < swerveModules.length; i++) {
      SwerveModule module = swerveModules[i];
      log.motor("module-" + i)
          .voltage(edu.wpi.first.units.Units.Volts.of(sysIdLastVoltage))
          .linearPosition(edu.wpi.first.units.Units.Meters.of(module.getDriveMotorPosition()))
          .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(module.getDriveMotorVelocity()));
    }
  }

  private void startSysId() {
    sysIdActive = true;
    robotSpeeds.stopSpeeds("drive");
    robotSpeeds.stopSpeeds("auto");
    robotSpeeds.stopSpeeds("feedback");
  }

  private void stopSysId() {
    sysIdActive = false;
    setChassisSpeeds(lastCommandedSpeeds, 0.0, 0.0, 0.0);
    for (SwerveModule module : swerveModules) {
      module.stop();
    }
  }

  private Command sysIdCommand(Supplier<Command> supplier) {
    return Commands.defer(
        () -> Commands.either(
            wrapSysIdCommand(supplier.get()),
            Commands.runOnce(() -> DriverStation.reportWarning("SysId commands require Test mode.", false)),
            DriverStation::isTest),
        Set.of(this));
  }

  private Command wrapSysIdCommand(Command base) {
    return Commands.runOnce(this::startSysId)
        .andThen(base)
        .finallyDo(this::stopSysId);
  }

  private double getVoltageLimit() {
    if (RobotBase.isSimulation()) {
      return nominalVoltage;
    }
    double battery = RobotController.getBatteryVoltage();
    if (Double.isFinite(battery) && battery > 1e-3) {
      return battery;
    }
    return nominalVoltage;
  }

  private void setNominalVoltage(double nominalVoltage) {
    if (Double.isFinite(nominalVoltage) && nominalVoltage > 1e-3) {
      this.nominalVoltage = nominalVoltage;
      for (SwerveModule module : swerveModules) {
        module.setNominalVoltage(nominalVoltage);
      }
    }
  }

  private static boolean angleNeedsUpdate(Rotation2d current, double desiredRadians) {
    if (current == null) {
      return true;
    }
    return absoluteAngularDifference(current.getRadians(), desiredRadians) > MODULE_ANGLE_UPDATE_EPSILON_RAD;
  }

  private static double absoluteAngularDifference(double aRadians, double bRadians) {
    return Math.abs(Math.IEEEremainder(aRadians - bRadians, 2.0 * Math.PI));
  }

  private MotionLimits.DriveLimits resolveDriveLimits() {
    MotionLimits.DriveLimits limits = motionLimits.resolveDriveLimits();
    return limits != null ? limits : MotionLimits.DriveLimits.none();
  }

  private ChassisSpeeds applyMotionLimits(ChassisSpeeds speeds, double nowSeconds) {
    MotionLimits.DriveLimits limits = resolveDriveLimits();
    ChassisSpeeds limited = limitVelocity(speeds, limits);
    limited = limitAcceleration(limited, limits, nowSeconds);
    setChassisSpeeds(
        lastLimitedSpeeds,
        limited.vxMetersPerSecond,
        limited.vyMetersPerSecond,
        limited.omegaRadiansPerSecond);
    return limited;
  }

  private void publishSwerveStatesIfDue(SwerveModuleState[] states, double nowSeconds) {
    if (Double.isFinite(lastSwerveStatePublishSeconds)
            && (nowSeconds - lastSwerveStatePublishSeconds) < SWERVE_STATE_PUBLISH_PERIOD_SECONDS) {
      return;
    }
    publisher.set(states);
    lastSwerveStatePublishSeconds = nowSeconds;
  }

  private ChassisSpeeds limitVelocity(ChassisSpeeds speeds, MotionLimits.DriveLimits limits) {
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double omega = speeds.omegaRadiansPerSecond;
    if (limits.maxLinearVelocity() > 0.0) {
      double linear = Math.hypot(vx, vy);
      if (linear > limits.maxLinearVelocity()) {
        double scale = limits.maxLinearVelocity() / linear;
        vx *= scale;
        vy *= scale;
      }
    }
    if (limits.maxAngularVelocity() > 0.0) {
      omega = MathUtil.clamp(omega, -limits.maxAngularVelocity(), limits.maxAngularVelocity());
    }
    setChassisSpeeds(velocityLimitedSpeeds, vx, vy, omega);
    return velocityLimitedSpeeds;
  }

  private ChassisSpeeds limitAcceleration(ChassisSpeeds speeds, MotionLimits.DriveLimits limits, double nowSeconds) {
    if (!Double.isFinite(lastMotionLimitTimestampSeconds)) {
      lastMotionLimitTimestampSeconds = nowSeconds;
      return speeds;
    }
    double dt = nowSeconds - lastMotionLimitTimestampSeconds;
    lastMotionLimitTimestampSeconds = nowSeconds;
    if (dt <= 0.0) {
      return speeds;
    }
    double maxLinearDelta = limits.maxLinearAcceleration() > 0.0
        ? limits.maxLinearAcceleration() * dt
        : Double.POSITIVE_INFINITY;
    double maxAngularDelta = limits.maxAngularAcceleration() > 0.0
        ? limits.maxAngularAcceleration() * dt
        : Double.POSITIVE_INFINITY;
    double vx = limitDelta(speeds.vxMetersPerSecond, lastLimitedSpeeds.vxMetersPerSecond, maxLinearDelta);
    double vy = limitDelta(speeds.vyMetersPerSecond, lastLimitedSpeeds.vyMetersPerSecond, maxLinearDelta);
    double omega = limitDelta(speeds.omegaRadiansPerSecond, lastLimitedSpeeds.omegaRadiansPerSecond, maxAngularDelta);
    setChassisSpeeds(accelerationLimitedSpeeds, vx, vy, omega);
    return accelerationLimitedSpeeds;
  }

  private static void setChassisSpeeds(ChassisSpeeds target, double vx, double vy, double omega) {
    target.vxMetersPerSecond = vx;
    target.vyMetersPerSecond = vy;
    target.omegaRadiansPerSecond = omega;
  }

  private static double nowSeconds() {
    double now = RobotTime.nowSeconds();
    if (!Double.isFinite(now)) {
      now = Timer.getFPGATimestamp();
    }
    return now;
  }

  private double limitDelta(double value, double lastValue, double maxDelta) {
    if (!Double.isFinite(maxDelta)) {
      return value;
    }
    double delta = value - lastValue;
    if (delta > maxDelta) {
      return lastValue + maxDelta;
    }
    if (delta < -maxDelta) {
      return lastValue - maxDelta;
    }
    return value;
  }

  public final class ControlRuntimeSection implements RobotDrivetrain.ControlSection {
    @Override
    public Command command(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
      return driveCommand(xInput, yInput, thetaInput);
    }

    @Override
    public void defaultCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
      setDefaultCommand(driveCommand(xInput, yInput, thetaInput));
    }

    @Override
    public void reset() {
      resetControlState();
    }

    @Override
    public void stop() {
      robotSpeeds.stop();
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
    }

    public boolean fieldRelative() {
      return fieldRelative;
    }

    public void fieldRelative(boolean enabled) {
      fieldRelative = enabled;
    }
  }

  public final class SpeedsRuntimeSection implements RobotDrivetrain.SpeedsSection {
    @Override
    public MotionLimits limits() {
      return limitsModel();
    }

    @Override
    public ChassisSpeeds get(String source) {
      return speedsModel().getSpeeds(source);
    }

    @Override
    public ChassisSpeeds getInput(String source) {
      return speedsModel().getInputSpeeds(source);
    }

    @Override
    public RobotDrivetrain.SpeedsSection set(String source, ChassisSpeeds speeds) {
      speedsModel().setSpeeds(source, speeds);
      return this;
    }

    @Override
    public RobotDrivetrain.SpeedsSection set(String source, double x, double y, double theta) {
      speedsModel().setSpeeds(source, x, y, theta);
      return this;
    }

    @Override
    public RobotDrivetrain.SpeedsSection stop(String source) {
      speedsModel().stopSpeeds(source);
      return this;
    }

    @Override
    public RobotDrivetrain.SpeedsSection stop() {
      speedsModel().stop();
      return this;
    }

    @Override
    public RobotDrivetrain.SpeedsSection enabled(String source, boolean enabled) {
      speedsModel().setSpeedSourceState(source, enabled);
      return this;
    }

    @Override
    public boolean enabled(String source) {
      return speedsModel().isSpeedsSourceActive(source);
    }

    @Override
    public double maxVelocity() {
      return speedsModel().getMaxVelocity();
    }

    @Override
    public double maxAngularVelocity() {
      return speedsModel().getMaxAngularVelocity();
    }

    @Override
    public Set<String> sources() {
      return speedsModel().getSpeedSources();
    }
  }

  public final class ModulesRuntimeSection implements RobotDrivetrain.ModulesSection {
    public SwerveModule[] all() {
      return swerveModules;
    }

    public SwerveModule module(int index) {
      return index >= 0 && index < swerveModules.length ? swerveModules[index] : null;
    }
  }

  public final class HardwareRuntimeSection implements RobotDrivetrain.HardwareSection {
    @Override
    public void neutralMode(MotorNeutralMode mode) {
      applyNeutralMode(mode);
    }
  }

  public final class SysIdRuntimeSection implements RobotDrivetrain.SysIdSection {
    @Override
    public double rampRateVoltsPerSecond() {
      return sysIdRampRateVoltsPerSecond();
    }

    @Override
    public void rampRateVoltsPerSecond(double voltsPerSecond) {
      sysIdRampRateVoltsPerSecond(voltsPerSecond);
    }

    @Override
    public double stepVoltage() {
      return sysIdStepVoltage();
    }

    @Override
    public void stepVoltage(double volts) {
      sysIdStepVoltage(volts);
    }

    @Override
    public double timeoutSeconds() {
      return sysIdTimeoutSeconds();
    }

    @Override
    public void timeoutSeconds(double seconds) {
      sysIdTimeoutSeconds(seconds);
    }

    @Override
    public boolean active() {
      return sysIdActive();
    }

    @Override
    public Command quasistatic(SysIdRoutine.Direction direction) {
      return sysIdCommand(() -> getSysIdRoutine().quasistatic(direction));
    }

    @Override
    public Command dynamic(SysIdRoutine.Direction direction) {
      return sysIdCommand(() -> getSysIdRoutine().dynamic(direction));
    }
  }

  public final class ImuRuntimeSection implements RobotDrivetrain.ImuSection {
    @Override
    public Imu device() {
      return imuDevice();
    }
  }

  public final class SimulationRuntimeSection implements RobotDrivetrain.SimulationSection {
    @Override
    public boolean enabled() {
      return simulationEnabled();
    }

    @Override
    public Pose2d pose() {
      return simulationPose();
    }

    @Override
    public void pose(Pose2d pose) {
      simulationPose(pose);
    }
  }
}
