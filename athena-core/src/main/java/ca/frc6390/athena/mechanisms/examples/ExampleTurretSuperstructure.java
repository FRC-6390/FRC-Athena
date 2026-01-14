package ca.frc6390.athena.mechanisms.examples;

import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.ArmMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.SuperstructureConfig;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Complete example showing a turret + hood + shooter superstructure using the mechanism config API.
 */
public final class ExampleTurretSuperstructure {

    private ExampleTurretSuperstructure() {
        /* utility */
    }

    public enum TurretState implements SetpointProvider<Double> {
        OFF(0.0),
        HUB(Math.toRadians(0.0)),
        NEUTRAL(Math.toRadians(90.0)),
        OPPONENT(Math.toRadians(180.0)),
        TARGET(0.0);

        private final double radians;

        TurretState(double radians) {
            this.radians = radians;
        }

        @Override
        public Double getSetpoint() {
            return radians;
        }
    }

    public enum HoodState implements SetpointProvider<Double> {
        STOW(Math.toRadians(0.0)),
        LOW(Math.toRadians(15.0)),
        MID(Math.toRadians(30.0)),
        HIGH(Math.toRadians(45.0));

        private final double radians;

        HoodState(double radians) {
            this.radians = radians;
        }

        @Override
        public Double getSetpoint() {
            return radians;
        }
    }

    public enum ShooterState implements SetpointProvider<Double> {
        OFF(0.0),
        SPINUP(rpm(3500.0)),
        FIRE(rpm(5000.0));

        private final double radiansPerSecond;

        ShooterState(double radiansPerSecond) {
            this.radiansPerSecond = radiansPerSecond;
        }

        @Override
        public Double getSetpoint() {
            return radiansPerSecond;
        }
    }

    private static double rpm(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    public record SuperSetpoint(TurretState turret, HoodState hood, ShooterState shooter) {
    }

    public enum SuperState implements SetpointProvider<SuperSetpoint> {
        STOWED(new SuperSetpoint(TurretState.NEUTRAL, HoodState.STOW, ShooterState.OFF)),
        TRACK(new SuperSetpoint(TurretState.TARGET, HoodState.MID, ShooterState.SPINUP)),
        FIRE(new SuperSetpoint(TurretState.TARGET, HoodState.MID, ShooterState.FIRE));

        private final SuperSetpoint setpoint;

        SuperState(SuperSetpoint setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public SuperSetpoint getSetpoint() {
            return setpoint;
        }
    }

    public static final class Handle {
        private final double[] targetHeadingRad;
        public final TurretMechanism.StatefulTurretMechanism<TurretState> turret;
        public final ArmMechanism.StatefulArmMechanism<HoodState> hood;
        public final FlywheelMechanism.StatefulFlywheelMechanism<ShooterState> shooter;
        public final SuperstructureMechanism<SuperState, SuperSetpoint> superstructure;

        private Handle(double[] targetHeadingRad,
                       TurretMechanism.StatefulTurretMechanism<TurretState> turret,
                       ArmMechanism.StatefulArmMechanism<HoodState> hood,
                       FlywheelMechanism.StatefulFlywheelMechanism<ShooterState> shooter,
                       SuperstructureMechanism<SuperState, SuperSetpoint> superstructure) {
            this.targetHeadingRad = targetHeadingRad;
            this.turret = turret;
            this.hood = hood;
            this.shooter = shooter;
            this.superstructure = superstructure;
        }

        public void setTarget(Pose2d robotPose, Translation2d target) {
            Rotation2d heading = turret.getFacingRotation(robotPose, target, Rotation2d.kZero);
            targetHeadingRad[0] = heading.getRadians();
        }

        public void requestTrack() {
            superstructure.getStateMachine().requestState(SuperState.TRACK);
        }

        public void requestFire() {
            superstructure.getStateMachine().requestState(SuperState.FIRE);
        }

        public void requestStowed() {
            superstructure.getStateMachine().requestState(SuperState.STOWED);
        }
    }

    public static Handle create(String canbus) {
        double[] targetHeadingRad = new double[1];

        MechanismConfig<TurretMechanism.StatefulTurretMechanism<TurretState>> turretConfig =
                MechanismConfig.statefulTurret(new SimpleMotorFeedforward(0.2, 0.1, 0.0), TurretState.NEUTRAL)
                        .addMotors(AthenaMotor.KRAKEN_X60, 1)
                        .setEncoder(new EncoderConfig()
                                .setType(AthenaEncoder.CANCODER.resolve())
                                .setGearRatio(100.0)
                                .setConversion(2.0 * Math.PI))
                        .setPID(new PIDController(2.0, 0.0, 0.1))
                        .setNeutralMode(MotorNeutralMode.Brake)
                        .setCanbus(canbus)
                        .setCurrentLimit(20)
                        .setBounds(Math.toRadians(-170.0), Math.toRadians(170.0))
                        .setSimulationSimpleMotor(new MechanismConfig.SimpleMotorSimulationParameters()
                                .setMomentOfInertia(0.02))
                        .setStateAction((TurretMechanism.StatefulTurretMechanism<TurretState> mech) -> {
                            mech.setSetpoint(targetHeadingRad[0]);
                            return false;
                        }, TurretState.TARGET);

        TurretMechanism.StatefulTurretMechanism<TurretState> turret = turretConfig.build();

        MechanismConfig<ArmMechanism.StatefulArmMechanism<HoodState>> hoodConfig =
                MechanismConfig.statefulArm(new ArmFeedforward(0.1, 0.2, 0.0, 0.0), HoodState.STOW)
                        .addMotors(AthenaMotor.NEO_V1, 2)
                        .setEncoder(new EncoderConfig()
                                .setType(AthenaEncoder.SPARK_MAX.resolve())
                                .setGearRatio(50.0)
                                .setConversion(2.0 * Math.PI))
                        .setPID(new PIDController(3.0, 0.0, 0.2))
                        .setNeutralMode(MotorNeutralMode.Brake)
                        .setCanbus(canbus)
                        .setCurrentLimit(15)
                        .setBounds(Math.toRadians(0.0), Math.toRadians(60.0))
                        .setSimulationArm(new MechanismConfig.ArmSimulationParameters()
                                .setArmLengthMeters(0.4)
                                .setStartingAngleRadians(0.0));

        ArmMechanism.StatefulArmMechanism<HoodState> hood = hoodConfig.build();

        MechanismConfig<FlywheelMechanism.StatefulFlywheelMechanism<ShooterState>> shooterConfig =
                MechanismConfig.statefulFlywheel(new SimpleMotorFeedforward(0.15, 0.12, 0.0), ShooterState.OFF)
                        .addMotors(AthenaMotor.KRAKEN_X44, 3)
                        .setEncoder(new EncoderConfig()
                                .setType(AthenaEncoder.CANCODER.resolve())
                                .setGearRatio(1.0)
                                .setConversion(2.0 * Math.PI))
                        .setPID(new PIDController(0.1, 0.0, 0.0))
                        .setPidUseVelocity(true)
                        .setUseSetpointAsOutput(false)
                        .setNeutralMode(MotorNeutralMode.Coast)
                        .setCanbus(canbus)
                        .setCurrentLimit(30)
                        .setBounds(0.0, rpm(6000.0))
                        .setSimulationSimpleMotor(new MechanismConfig.SimpleMotorSimulationParameters()
                                .setMomentOfInertia(0.01));

        FlywheelMechanism.StatefulFlywheelMechanism<ShooterState> shooter = shooterConfig.build();

        SuperstructureConfig<SuperState, SuperSetpoint> superConfig = SuperstructureConfig
                .create(SuperState.STOWED)
                .addExistingMechanism(turret, SuperSetpoint::turret)
                .addExistingMechanism(hood, SuperSetpoint::hood)
                .addExistingMechanism(shooter, SuperSetpoint::shooter);

        SuperstructureMechanism<SuperState, SuperSetpoint> superstructure = superConfig.build();

        return new Handle(targetHeadingRad, turret, hood, shooter, superstructure);
    }
}
