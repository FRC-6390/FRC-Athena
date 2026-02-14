package ca.frc6390.athena.drivetrains.swerve;

import java.util.Arrays;
import java.util.Set;
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

public class SwerveDrivetrain extends SubsystemBase implements RobotDrivetrain<SwerveDrivetrain> {

  private static final double DRIFT_TURN_DEADBAND_RAD_PER_SEC = 0.05;

  public SwerveModule[] swerveModules;
  public SwerveDriveKinematics kinematics;
  public PIDController driftpid;
  public boolean enableDriftCorrection, fieldRelative;
  public double desiredHeading, driftActivationSpeed;
  public Imu imu;
  public RobotSpeeds robotSpeeds;
  private final MotionLimits motionLimits;
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Drivetrain/SwerveStates", SwerveModuleState.struct).publish();
  private static final double SWERVE_STATE_PUBLISH_PERIOD_SECONDS = 0.1;
  private double lastSwerveStatePublishSeconds = Double.NaN;
  private SwerveDrivetrainSimulation simulation;
  private Field2d simulationField;
  private double lastSimulationTimestamp = -1;
  private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();
  private ChassisSpeeds lastLimitedSpeeds = new ChassisSpeeds();
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
  
  public SwerveDrivetrain(Imu imu, SwerveModuleConfig... modules) {

    this.imu = imu;

    enableDriftCorrection = false; 
    driftpid = new PIDController(0, 0,0);
    driftpid.enableContinuousInput(-Math.PI, Math.PI);

    swerveModules = new SwerveModule[modules.length];
    Translation2d[] moduleLocations = new Translation2d[swerveModules.length];
    double maxVelocity = 0;
    double minVelocity = Double.POSITIVE_INFINITY;
    double maxModuleRadius = 0;
    for (int i = 0; i < modules.length; i++) {
      swerveModules[i] = new SwerveModule(modules[i]);
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

    kinematics = new SwerveDriveKinematics(moduleLocations);
    double maxAngularVelocity = maxModuleRadius > 1e-6 ? maxVelocity / maxModuleRadius : maxVelocity;
    robotSpeeds = new RobotSpeeds(maxVelocity, maxAngularVelocity);
    motionLimits = new MotionLimits()
        .setBaseDriveLimits(MotionLimits.DriveLimits.fromRobotSpeeds(robotSpeeds));
    getIMU().addVirtualAxis("drift", () -> getIMU().getYaw());
    getIMU().setVirtualAxis("drift", getIMU().getVirtualAxis("driver"));
    desiredHeading = getIMU().getVirtualAxis("drift").getRadians();
    setNominalVoltage(nominalVoltage);
    
  }

  public SwerveDrivetrain withDriftCorretion(PIDController controller){
    driftpid = controller;
    driftpid.enableContinuousInput(-Math.PI, Math.PI);
    enableDriftCorrection = true;
    return this;
  }
  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  } 

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }


  private double driftCorrection(ChassisSpeeds speeds) {
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double commandedOmega = Math.abs(speeds.omegaRadiansPerSecond);
    double measuredOmega = Math.abs(imu.getVelocityZ().getRadians());

    // Hold heading only while translating straight; never fight intentional turns.
    if (commandedOmega > DRIFT_TURN_DEADBAND_RAD_PER_SEC
        || measuredOmega > DRIFT_TURN_DEADBAND_RAD_PER_SEC
        || linearSpeed < Math.max(0.0, driftActivationSpeed)) {
      desiredHeading = getIMU().getVirtualAxis("drift").getRadians();
      return 0;
    } else {
      return driftpid.calculate(getIMU().getVirtualAxis("drift").getRadians(), desiredHeading);
    }
  }

  public SwerveDrivetrain setDriftCorrectionPID(PIDController controller) {
    driftpid = controller;
    return this;
  }
  
  public void setDriftCorrectionMode(boolean enabled) {
    enableDriftCorrection = enabled;
  }

  public boolean getDriftCorrectionMode() {
    return enableDriftCorrection;
  }

  public void setFieldRelative(boolean fieldRelative) {
      this.fieldRelative = fieldRelative;
  }

  public SwerveDrivetrain setDriveFeedforward(SimpleMotorFeedforward feedforward) {
    if (feedforward == null) {
      driveFeedforward = null;
      setDriveFeedforwardEnabled(false);
      Arrays.stream(swerveModules).forEach(module -> module.setDriveFeedforward(null));
      return this;
    }
    driveFeedforward = new SimpleMotorFeedForwardsSendable(
        feedforward.getKs(),
        feedforward.getKv(),
        feedforward.getKa());
    Arrays.stream(swerveModules).forEach(module -> module.setDriveFeedforward(driveFeedforward));
    setDriveFeedforwardEnabled(true);
    return this;
  }

  public void setDriveFeedforwardEnabled(boolean enabled) {
    driveFeedforwardEnabled = driveFeedforward != null && enabled;
    Arrays.stream(swerveModules).forEach(module -> module.setDriveFeedforwardEnabled(driveFeedforwardEnabled));
  }

  public boolean isDriveFeedforwardEnabled() {
    return driveFeedforwardEnabled;
  }

  public boolean hasDriveFeedforward() {
    return driveFeedforward != null;
  }

  public double getSysIdRampRateVoltsPerSecond() {
    return sysIdRampRateVoltsPerSecond;
  }

  public void setSysIdRampRateVoltsPerSecond(double rampRate) {
    if (!Double.isFinite(rampRate) || rampRate <= 0.0) {
      return;
    }
    sysIdRampRateVoltsPerSecond = rampRate;
    invalidateSysIdRoutine();
  }

  public double getSysIdStepVoltage() {
    return sysIdStepVoltage;
  }

  public void setSysIdStepVoltage(double stepVoltage) {
    if (!Double.isFinite(stepVoltage) || stepVoltage <= 0.0) {
      return;
    }
    sysIdStepVoltage = stepVoltage;
    invalidateSysIdRoutine();
  }

  public double getSysIdTimeoutSeconds() {
    return sysIdTimeoutSeconds;
  }

  public void setSysIdTimeoutSeconds(double timeoutSeconds) {
    if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0.0) {
      return;
    }
    sysIdTimeoutSeconds = timeoutSeconds;
    invalidateSysIdRoutine();
  }

  public boolean isSysIdActive() {
    return sysIdActive;
  }

  @Override
  public Imu getIMU() {
      return imu;
  }

  @Override
  public void setNeutralMode(MotorNeutralMode mode) {
    Arrays.stream(swerveModules).forEach(m -> m.setNeutralMode(mode));
    // for (int i = 0; i < swerveModules.length; i++) {
    //   swerveModules[i].setNeutralMode(mode);
    // }
  }

  @Override
  public RobotSpeeds getRobotSpeeds() {
    return robotSpeeds;
  }

  @Override
  public MotionLimits getMotionLimits() {
    return motionLimits;
  }

  @Override
  public void resetDriveState() {
    robotSpeeds.stop();
    lastCommandedSpeeds = new ChassisSpeeds();
    lastLimitedSpeeds = new ChassisSpeeds();
    lastMotionLimitTimestampSeconds = Double.NaN;
    desiredHeading = getIMU().getVirtualAxis("drift").getRadians();
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

    ChassisSpeeds speed = robotSpeeds.calculate();

    if (enableDriftCorrection) {
      speed.omegaRadiansPerSecond += driftCorrection(speed);
    }
    speed = applyMotionLimits(speed);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    publishSwerveStatesIfDue(states);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, getRobotSpeeds().getMaxVelocity());

    setModuleStates(states);   

    lastCommandedSpeeds = new ChassisSpeeds(
        speed.vxMetersPerSecond,
        speed.vyMetersPerSecond,
        speed.omegaRadiansPerSecond
    );
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
    simulationField.setRobotPose(new Pose2d());
    lastSimulationTimestamp = -1;
    return this;
  }

  public boolean hasSimulation() {
    return simulation != null;
  }

  public Pose2d getSimulatedPose() {
    return simulation != null ? simulation.getPose() : new Pose2d();
  }

  public void resetSimulationPose(Pose2d pose) {
    if (simulation != null) {
      simulation.resetPose(pose);
    }
  }

  @Override
  public Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    return new SwerveDriveCommand(this, xInput, yInput, thetaInput, fieldRelative);
  }

  @Override
  public void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    this.setDefaultCommand(getDriveCommand(xInput, yInput, thetaInput));
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

    RobotNetworkTables.Node modules = node.child("SwerveModules");
    for (int i = 0; i < swerveModules.length; i++) {
      if (swerveModules[i] != null) {
        swerveModules[i].networkTables(modules.child("Module" + i));
      }
    }

    RobotNetworkTables.Node drift = node.child("DriftCorrection");
    drift.putBoolean("enabled", getDriftCorrectionMode());
    drift.putDouble("desiredHeadingDeg", desiredHeading);

    RobotNetworkTables.Node sysid = node.child("SysId");
    sysid.putDouble("rampRateVPerSec", getSysIdRampRateVoltsPerSecond());
    sysid.putDouble("stepVoltageV", getSysIdStepVoltage());
    sysid.putDouble("timeoutSec", getSysIdTimeoutSeconds());
    sysid.putBoolean("active", isSysIdActive());

    if (simulation != null) {
      Pose2d pose = getSimulatedPose();
      RobotNetworkTables.Node simNode = node.child("Sim");
      simNode.putDouble("xM", pose.getX());
      simNode.putDouble("yM", pose.getY());
      simNode.putDouble("rotDeg", pose.getRotation().getDegrees());
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
        double now = Timer.getFPGATimestamp();
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
  public SwerveDrivetrain get() {
    return this;
  }

  @Override
  public RobotLocalization<SwerveModulePosition[]> localization(RobotLocalizationConfig config) {
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
                          getPositions(),
                          startPose,
                          stateStdDevs,
                          visionStdDevs);
          estimator.setSlipThreshold(backend.slipThreshold());
          return estimator;
        }
        return new SwerveDrivePoseEstimator(
                kinematics,
                imu.getYaw(),
                getPositions(),
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
                getPositions(),
                startPose,
                stateStdDevs,
                visionStdDevs);
      }
    };

    return new RobotLocalization<>(factory, effectiveConfig, robotSpeeds, imu, this::getPositions);
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
    lastCommandedSpeeds = new ChassisSpeeds();
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
      Arrays.stream(swerveModules).forEach(module -> module.setNominalVoltage(nominalVoltage));
    }
  }

  private MotionLimits.DriveLimits resolveDriveLimits() {
    MotionLimits.DriveLimits limits = motionLimits.resolveDriveLimits();
    return limits != null ? limits : MotionLimits.DriveLimits.none();
  }

  private ChassisSpeeds applyMotionLimits(ChassisSpeeds speeds) {
    MotionLimits.DriveLimits limits = resolveDriveLimits();
    ChassisSpeeds limited = limitVelocity(speeds, limits);
    double nowSeconds = RobotTime.nowSeconds();
    if (!Double.isFinite(nowSeconds)) {
      nowSeconds = Timer.getFPGATimestamp();
    }
    limited = limitAcceleration(limited, limits, nowSeconds);
    lastLimitedSpeeds = limited;
    return limited;
  }

  private void publishSwerveStatesIfDue(SwerveModuleState[] states) {
    double nowSeconds = RobotTime.nowSeconds();
    if (!Double.isFinite(nowSeconds)) {
      nowSeconds = Timer.getFPGATimestamp();
    }
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
    return new ChassisSpeeds(vx, vy, omega);
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
    return new ChassisSpeeds(vx, vy, omega);
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
}
