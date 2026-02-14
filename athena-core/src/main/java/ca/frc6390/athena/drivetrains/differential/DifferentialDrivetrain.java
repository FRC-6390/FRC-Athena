package ca.frc6390.athena.drivetrains.differential;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.commands.control.TankDriveCommand;
import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.core.localization.PoseEstimatorFactory;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.localization.RobotDrivetrainLocalizationFactory;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.controllers.SimpleMotorFeedForwardsSendable;
import ca.frc6390.athena.drivetrains.differential.sim.DifferentialDrivetrainSimulation;
import ca.frc6390.athena.drivetrains.differential.sim.DifferentialSimulationConfig;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDrivetrain extends SubsystemBase
        implements RobotDrivetrain<DifferentialDrivetrain>, RobotDrivetrainLocalizationFactory {

    private final RobotSpeeds robotSpeeds;
    private final MotionLimits motionLimits;
    private final ControlRuntimeSection controlSection = new ControlRuntimeSection();
    private final SpeedsRuntimeSection speedsSection = new SpeedsRuntimeSection();
    private final ModulesRuntimeSection modulesSection = new ModulesRuntimeSection();
    private final HardwareRuntimeSection hardwareSection = new HardwareRuntimeSection();
    private final SysIdRuntimeSection sysIdSection = new SysIdRuntimeSection();
    private final ImuRuntimeSection imuSection = new ImuRuntimeSection();
    private final SimulationRuntimeSection simulationSection = new SimulationRuntimeSection();
    private final Imu imu;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrive drive;
    private final MotorControllerGroup leftMotors, rightMotors;
    private final EncoderGroup leftEncoders, rightEncoders;
    private DifferentialDrivetrainSimulation simulation;
    private Field2d simulationField;
    private double lastSimulationTimestamp = -1;
    private double lastLeftCommand = 0;
    private double lastRightCommand = 0;
    private ChassisSpeeds lastLimitedSpeeds = new ChassisSpeeds();
    private double lastMotionLimitTimestampSeconds = Double.NaN;
    private SimpleMotorFeedForwardsSendable driveFeedforward;
    private boolean driveFeedforwardEnabled = false;
    private double nominalVoltage = 12.0;
    private double lastFeedforwardTimestampSeconds = Double.NaN;
    private double lastLeftSetpointMetersPerSecond = 0.0;
    private double lastRightSetpointMetersPerSecond = 0.0;
    private SysIdRoutine sysIdRoutine;
    private double sysIdRampRateVoltsPerSecond = 1.0;
    private double sysIdStepVoltage = 7.0;
    private double sysIdTimeoutSeconds = 10.0;
    private double sysIdLastVoltage = 0.0;
    private boolean sysIdActive = false;

    public DifferentialDrivetrain(Imu imu, double maxVelocity, double trackwidth, MotorControllerGroup leftMotors, MotorControllerGroup rightMotors){
       this(imu, maxVelocity, trackwidth, leftMotors, rightMotors, leftMotors.getEncoderGroup(), rightMotors.getEncoderGroup());
    }

    public DifferentialDrivetrain(Imu imu, double maxVelocity, double trackwidth, MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, EncoderGroup leftEncoders, EncoderGroup rightEncoders){
        this.imu = imu;
        robotSpeeds = new RobotSpeeds(maxVelocity, maxVelocity);
        motionLimits = new MotionLimits()
                .setBaseDriveLimits(MotionLimits.DriveLimits.fromRobotSpeeds(robotSpeeds));

        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;

        this.rightEncoders = rightEncoders;
        this.leftEncoders = leftEncoders;

        this.drive = new DifferentialDrive(this::setLeftOutput, this::setRightOutput);
        this.kinematics = new DifferentialDriveKinematics(trackwidth);
    }

    @Override
    public DifferentialDrivetrain control(Consumer<RobotDrivetrain.ControlSection> section) {
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
    public DifferentialDrivetrain speeds(Consumer<RobotDrivetrain.SpeedsSection> section) {
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
    public DifferentialDrivetrain modules(Consumer<RobotDrivetrain.ModulesSection> section) {
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
    public DifferentialDrivetrain hardware(Consumer<RobotDrivetrain.HardwareSection> section) {
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
    public DifferentialDrivetrain sysId(Consumer<RobotDrivetrain.SysIdSection> section) {
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
    public DifferentialDrivetrain imu(Consumer<RobotDrivetrain.ImuSection> section) {
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
    public DifferentialDrivetrain simulation(Consumer<RobotDrivetrain.SimulationSection> section) {
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

    private Imu imuDevice() {
        return imu;
    }

    private void applyNeutralMode(MotorNeutralMode mode) {
       leftMotors.setNeutralMode(mode);
       rightMotors.setNeutralMode(mode);
    }

    private RobotSpeeds speedsModel() {
        return robotSpeeds;
    }

    private MotionLimits limitsModel() {
        return motionLimits;
    }

    private void resetControlState() {
        robotSpeeds.stop();
        lastLimitedSpeeds = new ChassisSpeeds();
        lastMotionLimitTimestampSeconds = Double.NaN;
        lastLeftCommand = 0.0;
        lastRightCommand = 0.0;
        resetFeedforwardState();
        leftMotors.stopMotors();
        rightMotors.stopMotors();
    }

    public DifferentialDrivetrain driveFeedforward(SimpleMotorFeedforward feedforward) {
        if (feedforward == null) {
            driveFeedforward = null;
            driveFeedforwardEnabled(false);
            return this;
        }
        driveFeedforward = new SimpleMotorFeedForwardsSendable(
                feedforward.getKs(),
                feedforward.getKv(),
                feedforward.getKa());
        driveFeedforwardEnabled(true);
        return this;
    }

    public void driveFeedforwardEnabled(boolean enabled) {
        driveFeedforwardEnabled = driveFeedforward != null && enabled;
        if (!driveFeedforwardEnabled) {
            resetFeedforwardState();
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

    @Override
    public void update() {
        if (imu != null) {
            imu.update();
        }
        leftMotors.update();
        rightMotors.update();
        if (simulation == null) {
            if (leftEncoders != null) {
                leftEncoders.update();
            }
            if (rightEncoders != null) {
                rightEncoders.update();
            }
        }
        
        if (sysIdActive) {
            return;
        }

        ChassisSpeeds speeds = robotSpeeds.calculate();
        speeds = applyMotionLimits(speeds);
        if (driveFeedforwardEnabled && driveFeedforward != null) {
            DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
            double now = Timer.getFPGATimestamp();
            double leftCurrent = Double.isFinite(lastFeedforwardTimestampSeconds)
                    ? lastLeftSetpointMetersPerSecond
                    : wheelSpeeds.leftMetersPerSecond;
            double rightCurrent = Double.isFinite(lastFeedforwardTimestampSeconds)
                    ? lastRightSetpointMetersPerSecond
                    : wheelSpeeds.rightMetersPerSecond;
            lastFeedforwardTimestampSeconds = now;
            lastLeftSetpointMetersPerSecond = wheelSpeeds.leftMetersPerSecond;
            lastRightSetpointMetersPerSecond = wheelSpeeds.rightMetersPerSecond;

            double leftVolts = driveFeedforward.calculateWithVelocities(leftCurrent, wheelSpeeds.leftMetersPerSecond);
            double rightVolts = driveFeedforward.calculateWithVelocities(rightCurrent, wheelSpeeds.rightMetersPerSecond);
            setLeftVoltage(leftVolts);
            setRightVoltage(rightVolts);
            drive.feed();
        } else {
            drive.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
    }

    private Command driveCommand(DoubleSupplier xInput, DoubleSupplier thetaInput){
        return new TankDriveCommand(this, xInput, thetaInput);
    }

    private Command driveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
        return driveCommand(xInput, thetaInput);
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
            simulation.update(dt > 0 ? dt : 0.02, lastLeftCommand, lastRightCommand);
            if (simulationField != null) {
                simulationField.setRobotPose(simulation.getPose());
            }
        }
    }

    private DifferentialDriveWheelPositions positions(){
     double leftPosition = leftEncoders != null ? leftEncoders.getPosition() : 0;
     double rightPosition = rightEncoders != null ? rightEncoders.getPosition() : 0;
     return new DifferentialDriveWheelPositions(leftPosition, rightPosition);
    }

     @Override
    public RobotLocalization<DifferentialDriveWheelPositions> createLocalization(RobotLocalizationConfig config) {
        RobotLocalizationConfig effectiveConfig = config != null ? config : RobotLocalizationConfig.defaults();
        PoseEstimatorFactory<DifferentialDriveWheelPositions> factory = new PoseEstimatorFactory<>() {
            @Override
            public DifferentialDrivePoseEstimator create2d(
                    Pose2d startPose,
                    Matrix<N3, N1> stateStdDevs,
                    Matrix<N3, N1> visionStdDevs) {
                DifferentialDriveWheelPositions positions = positions();
                return new DifferentialDrivePoseEstimator(
                        kinematics,
                        imu.getYaw(),
                        positions.leftMeters,
                        positions.rightMeters,
                        startPose,
                        stateStdDevs,
                        visionStdDevs);
            }

            @Override
            public DifferentialDrivePoseEstimator3d create3d(
                    Pose3d startPose,
                    Matrix<N4, N1> stateStdDevs,
                    Matrix<N4, N1> visionStdDevs) {
                DifferentialDriveWheelPositions positions = positions();
                Rotation3d gyroRotation = getImuRotation3d();
                return new DifferentialDrivePoseEstimator3d(
                        kinematics,
                        gyroRotation,
                        positions.leftMeters,
                        positions.rightMeters,
                        startPose,
                        stateStdDevs,
                        visionStdDevs);
            }
        };

        return new RobotLocalization<>(factory, effectiveConfig, robotSpeeds, imu, this::positions);
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        RobotDrivetrain.super.networkTables(node);

        if (driveFeedforward != null && nt.enabled(RobotNetworkTables.Flag.DRIVETRAIN_SPEED_WIDGETS)) {
            RobotNetworkTables.Node ff = node.child("DriveFeedforward");
            ff.putBoolean("enabled", driveFeedforwardEnabled());
        }

        RobotNetworkTables.Node sysid = node.child("SysId");
        sysid.putDouble("rampRateVPerSec", sysIdRampRateVoltsPerSecond());
        sysid.putDouble("stepVoltageV", sysIdStepVoltage());
        sysid.putDouble("timeoutSec", sysIdTimeoutSeconds());
        sysid.putBoolean("active", sysIdActive());

        if (simulation != null) {
            Pose2d pose = simulationPose();
            RobotNetworkTables.Node simNode = node.child("Sim");
            simNode.putDouble("xM", pose.getX());
            simNode.putDouble("yM", pose.getY());
            simNode.putDouble("rotDeg", pose.getRotation().getDegrees());
        }

        return node;
    }

    public DifferentialDrivetrain configureSimulation(DifferentialSimulationConfig config) {
        if (!RobotBase.isSimulation()) {
            return this;
        }
        simulation = new DifferentialDrivetrainSimulation(this, config, leftEncoders, rightEncoders);
        if (config != null) {
            nominalVoltage = config.getNominalVoltage();
        }
        if (simulationField == null) {
            simulationField = new Field2d();
        }
        simulationField.setRobotPose(simulation.getPose());
        lastSimulationTimestamp = -1;
        return this;
    }

    private boolean simulationEnabled() {
        return simulation != null;
    }

    private Pose2d simulationPose() {
        return simulation != null ? simulation.getPose() : new Pose2d();
    }

    private void simulationPose(Pose2d pose) {
        if (simulation != null) {
            simulation.resetPose(pose);
        }
    }

    private Rotation3d getImuRotation3d() {
        return new Rotation3d(
                imu.getRoll().getRadians(),
                imu.getPitch().getRadians(),
                imu.getYaw().getRadians());
    }

    private void setLeftOutput(double output) {
        lastLeftCommand = output;
        leftMotors.setSpeed(output);
    }

    private void setRightOutput(double output) {
        lastRightCommand = output;
        rightMotors.setSpeed(output);
    }

    private void setLeftVoltage(double volts) {
        double voltageLimit = getVoltageLimit();
        double clamped = MathUtil.clamp(volts, -voltageLimit, voltageLimit);
        lastLeftCommand = MathUtil.clamp(clamped / voltageLimit, -1.0, 1.0);
        leftMotors.setVoltage(clamped);
    }

    private void setRightVoltage(double volts) {
        double voltageLimit = getVoltageLimit();
        double clamped = MathUtil.clamp(volts, -voltageLimit, voltageLimit);
        lastRightCommand = MathUtil.clamp(clamped / voltageLimit, -1.0, 1.0);
        rightMotors.setVoltage(clamped);
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

    private void resetFeedforwardState() {
        lastFeedforwardTimestampSeconds = Double.NaN;
        lastLeftSetpointMetersPerSecond = 0.0;
        lastRightSetpointMetersPerSecond = 0.0;
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
        setLeftVoltage(applied);
        setRightVoltage(applied);
        drive.feed();
    }

    private void logSysIdData(SysIdRoutineLog log) {
        double leftPosition = leftEncoders != null ? leftEncoders.getPosition() : 0.0;
        double rightPosition = rightEncoders != null ? rightEncoders.getPosition() : 0.0;
        double leftVelocity = leftEncoders != null ? leftEncoders.getVelocity() : 0.0;
        double rightVelocity = rightEncoders != null ? rightEncoders.getVelocity() : 0.0;

        log.motor("left")
                .voltage(edu.wpi.first.units.Units.Volts.of(sysIdLastVoltage))
                .linearPosition(edu.wpi.first.units.Units.Meters.of(leftPosition))
                .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(leftVelocity));

        log.motor("right")
                .voltage(edu.wpi.first.units.Units.Volts.of(sysIdLastVoltage))
                .linearPosition(edu.wpi.first.units.Units.Meters.of(rightPosition))
                .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(rightVelocity));
    }

    private void startSysId() {
        sysIdActive = true;
        robotSpeeds.stopSpeeds("drive");
        robotSpeeds.stopSpeeds("auto");
        robotSpeeds.stopSpeeds("feedback");
    }

    private void stopSysId() {
        sysIdActive = false;
        leftMotors.stopMotors();
        rightMotors.stopMotors();
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

    private MotionLimits.DriveLimits resolveDriveLimits() {
        MotionLimits.DriveLimits limits = motionLimits.resolveDriveLimits();
        return limits != null ? limits : MotionLimits.DriveLimits.none();
    }

    private ChassisSpeeds applyMotionLimits(ChassisSpeeds speeds) {
        MotionLimits.DriveLimits limits = resolveDriveLimits();
        ChassisSpeeds limited = limitVelocity(speeds, limits);
        limited = limitAcceleration(limited, limits, Timer.getFPGATimestamp());
        lastLimitedSpeeds = limited;
        return limited;
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
            leftMotors.stopMotors();
            rightMotors.stopMotors();
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
        public MotorControllerGroup leftMotors() {
            return leftMotors;
        }

        public MotorControllerGroup rightMotors() {
            return rightMotors;
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
