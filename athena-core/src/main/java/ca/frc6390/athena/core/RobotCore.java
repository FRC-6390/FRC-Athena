package ca.frc6390.athena.core;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.sim.RobotVisionSim;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanism;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCameraCapability;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotCore<T extends RobotDrivetrain<T>> extends TimedRobot {

    public record RobotCoreConfig<T extends RobotDrivetrain<T>>(RobotDrivetrainConfig<T> driveTrain,
            RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig) {

        public static RobotCoreConfig<SwerveDrivetrain> swerve(SwerveDrivetrainConfig config) {
            return new RobotCoreConfig<>(config, RobotLocalizationConfig.defualt(), RobotVisionConfig.defualt());
        }

        public static RobotCoreConfig<DifferentialDrivetrain> differential(DifferentialDrivetrainConfig config) {
            return new RobotCoreConfig<>(config, RobotLocalizationConfig.defualt(), RobotVisionConfig.defualt());
        }

        public RobotCoreConfig<T> setLocalization(RobotLocalizationConfig localizationConfig) {
            return new RobotCoreConfig<>(driveTrain, localizationConfig, visionConfig);
        }

        public RobotCoreConfig<T> setVision(RobotVisionConfig visionConfig) {
            return new RobotCoreConfig<>(driveTrain, localizationConfig, visionConfig);
        }

        public RobotCoreConfig<T> setVision(ConfigurableCamera... cameras) {
            return new RobotCoreConfig<>(driveTrain, localizationConfig,
                    RobotVisionConfig.defualt().addCameras(cameras));
        }

        public RobotCore<T> create() {
            return new RobotCore<>(this);
        }
    }

    private final RobotDrivetrain<T> drivetrain;
    private final RobotLocalization<?> localization;
    private final RobotVision vision;
    private RobotVisionSim visionSim;
    private Notifier visionSimNotifier;
    private final RobotAuto autos;
    private final HashMap<String, Mechanism> mechanisms;
    private final Set<Mechanism> scheduledCustomPidMechanisms;
    private Command autonomousCommand;

    public RobotCore(RobotCoreConfig<T> config) {
        drivetrain = config.driveTrain.build();
        localization = drivetrain.localization(config.localizationConfig());
        vision = RobotVision.fromConfig(config.visionConfig);
        autos = new RobotAuto();
        mechanisms = new HashMap<>();
        scheduledCustomPidMechanisms = new HashSet<>();
        autonomousCommand = null;

        if (localization != null && vision != null) {
            localization.setRobotVision(vision);
            localization.configureChoreo(drivetrain);
        }

        if (vision != null && edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            AprilTagFieldLayout layout = vision.deriveSimulationLayout();
            visionSim = vision.createSimulation(layout);
            visionSimNotifier = new Notifier(() -> {
                Pose2d pose = localization != null ? localization.getFieldPose() : new Pose2d();
                visionSim.update(pose);
            });
            visionSimNotifier.startPeriodic(0.02);
        }
    }

    @Override
    public final void robotInit() {
        configureAutos(autos);
        ensureAutoChooserPublished();
        onRobotInit();
        registerPIDCycles();
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
        onRobotPeriodic();
    }

    @Override
    public final void autonomousInit() {
        scheduleAutonomousCommand();
        onAutonomousInit();
    }

    @Override
    public final void autonomousExit() {
        onAutonomousExit();
    }

    @Override
    public final void autonomousPeriodic() {
        onAutonomousPeriodic();
    }

    @Override
    public final void teleopInit() {
        cancelAutonomousCommand();
        onTeleopInit();
    }

    @Override
    public final void teleopExit() {
        onTeleopExit();
    }

    @Override
    public final void teleopPeriodic() {
        onTeleopPeriodic();
    }

    @Override
    public final void disabledInit() {
        cancelAutonomousCommand();
        onDisabledInit();
    }

    @Override
    public final void disabledExit() {
        onDisabledExit();
    }

    @Override
    public final void disabledPeriodic() {
        onDisabledPeriodic();
    }

    @Override
    public final void testInit() {
        CommandScheduler.getInstance().cancelAll();
        cancelAutonomousCommand();
        onTestInit();
    }

    @Override
    public final void testExit() {
        onTestExit();
    }

    @Override
    public final void testPeriodic() {
        onTestPeriodic();
    }

    @Override
    public final void simulationInit() {
        onSimulationInit();
    }

    @Override
    public final void simulationPeriodic() {
        onSimulationPeriodic();
    }

    protected void onRobotInit() {}

    protected void onRobotPeriodic() {}

    protected void onAutonomousInit() {}

    protected void onAutonomousExit() {}

    protected void onAutonomousPeriodic() {}

    protected void onTeleopInit() {}

    protected void onTeleopExit() {}

    protected void onTeleopPeriodic() {}

    protected void onDisabledInit() {}

    protected void onDisabledExit() {}

    protected void onDisabledPeriodic() {}

    protected void onTestInit() {}

    protected void onTestExit() {}

    protected void onTestPeriodic() {}

    protected void onSimulationInit() {}

    protected void onSimulationPeriodic() {}

    /**
     * Hook for registering autonomous routines and named commands.
     * Called once during {@link #robotInit()} after all subsystems are constructed.
     */
    protected void configureAutos(RobotAuto auto) {}

    protected Command createAutonomousCommand() {
        return autos.buildSelectedCommand().orElse(null);
    }

    private void scheduleAutonomousCommand() {
        cancelAutonomousCommand();
        Command selected = createAutonomousCommand();
        if (selected != null) {
            CommandScheduler.getInstance().schedule(selected);
            autonomousCommand = selected;
        }
    }

    private void cancelAutonomousCommand() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        getDrivetrain().getRobotSpeeds().stopSpeeds("auto");
    }

    private void scheduleCustomPidCycle(Mechanism mechanism) {
        if (!mechanism.isCustomPIDCycle()) {
            return;
        }
        if (!scheduledCustomPidMechanisms.add(mechanism)) {
            return;
        }
        double period = mechanism.getPidPeriod();
        if (!(period > 0.0)) {
            period = 0.02;
        }
        addPeriodic(mechanism::updatePID, period);
    }

    public RobotCore<T> registerMechanism(Mechanism... mechs) {
        Arrays.stream(mechs).forEach(mech -> {
            mechanisms.put(mech.getName(), mech);
            scheduleCustomPidCycle(mech);
        });
        return this;
    }

    /**
     * Registers all child mechanisms contained within the given superstructures.
     */
    public RobotCore<T> registerMechanism(SuperstructureMechanism<?, ?>... supers) {
        Arrays.stream(supers).forEach(s -> registerMechanism(s.getMechanisms().all().toArray(Mechanism[]::new)));
        return this;
    }

    /**
     * Registers any registerable mechanism (plain or composite).
     */
    public RobotCore<T> registerMechanism(RegisterableMechanism... entries) {
        Arrays.stream(entries).forEach(entry -> registerMechanism(entry.flattenForRegistration().toArray(Mechanism[]::new)));
        return this;
    }

    public RobotLocalization<?> getLocalization() {
        return localization;
    }

    public T getDrivetrain() {
        return drivetrain.get();
    }

    public RobotVision getVision() {
        return vision;
    }

    public RobotCore<T> shuffleboard() {
        return shuffleboard("Drivetrain");
    }

    public RobotCore<T> shuffleboard(String drive) {
        return shuffleboard(drive, "Localization");
    }

    public RobotCore<T> shuffleboard(SendableLevel level) {
        return shuffleboard("Drivetrain", level);
    }

    public RobotCore<T> shuffleboard(String drive, SendableLevel level) {
        return shuffleboard(drive, "Localization", level);
    }

    public RobotCore<T> shuffleboard(String drive, String local) {
        return shuffleboard(drive, local, SendableLevel.COMP);
    }

    public RobotCore<T> shuffleboard(String drive, String local, SendableLevel level) {
        drivetrain.shuffleboard(drive, level);

        if (localization != null) {
            localization.shuffleboard(local, level);
        }

        if (vision != null) {
            vision.shuffleboard("Robot Vision", level);
        }

        return this;
    }

    public RotateToPoint rotateTo(double x, double y) {
        return new RotateToPoint(this, x, y);
    }

    public RotateToAngle rotateTo(double degrees) {
        return new RotateToAngle(this, degrees);
    }

    public RotateToAngle rotateBy(double degrees) {
        return new RotateToAngle(
                this,
                getDrivetrain().getIMU().getYaw().plus(Rotation2d.fromDegrees(degrees)));
    }

    public RobotSpeeds getRobotSpeeds() {
        return drivetrain.getRobotSpeeds();
    }

    public Imu getIMU() {
        return drivetrain.getIMU();
    }

    public SendableChooser<Command> registerAutoChooser(RobotAuto.AutoKey defaultAuto) {
        SendableChooser<Command> chooser = autos.createCommandChooser(defaultAuto);
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoRoutineChooser(RobotAuto.AutoKey defaultAuto) {
        SendableChooser<RobotAuto.AutoRoutine> chooser = autos.createChooser(defaultAuto);
        SmartDashboard.putData("Auto Routine Chooser", chooser);
        return chooser;
    }

    public SendableChooser<RobotAuto.AutoRoutine> registerAutoRoutineChooser(String defaultAuto) {
        return registerAutoRoutineChooser(RobotAuto.AutoKey.of(defaultAuto));
    }

    public SendableChooser<Command> registerAutoChooser(String defualtAuto) {
        return registerAutoChooser(RobotAuto.AutoKey.of(defualtAuto));
    }

    public RobotAuto getAutos() {
        return autos;
    }

    private void ensureAutoChooserPublished() {
        if (autos.getAutoChooser() != null || autos.getCommandChooser() != null) {
            return;
        }
        autos.getAutos().stream()
                .findFirst()
                .map(RobotAuto.AutoRoutine::key)
                .ifPresent(this::registerAutoChooser);
    }

    public void registerPIDCycles() {
        mechanisms.values().forEach(this::scheduleCustomPidCycle);
    }

    public void resetPIDs() {
        for (Mechanism mech : mechanisms.values()) {
            mech.resetPID();
        }
    }

    public ChassisSpeeds createRobotRelativeSpeeds(double xSpeed, double ySpeed, double rot) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, rot), getLocalization().getRelativePose().getRotation());
    }

    public ChassisSpeeds createFieldRelativeSpeeds(double xSpeed, double ySpeed, double rot) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, rot), getLocalization().getRelativePose().getRotation());
    }
}
