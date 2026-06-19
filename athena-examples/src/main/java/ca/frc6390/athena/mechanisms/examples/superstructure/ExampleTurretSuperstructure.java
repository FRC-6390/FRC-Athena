package ca.frc6390.athena.mechanisms.examples;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.superstructure.SuperstructureDefinitions;
import ca.frc6390.athena.api.superstructure.Superstructures;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
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
        public final StatefulMechanism<TurretState> turret;
        public final StatefulMechanism<HoodState> hood;
        public final StatefulMechanism<ShooterState> shooter;
        public final SuperstructureMechanism<SuperState, SuperSetpoint> superstructure;

        private Handle(double[] targetHeadingRad,
                       StatefulMechanism<TurretState> turret,
                       StatefulMechanism<HoodState> hood,
                       StatefulMechanism<ShooterState> shooter,
                       SuperstructureMechanism<SuperState, SuperSetpoint> superstructure) {
            this.targetHeadingRad = targetHeadingRad;
            this.turret = turret;
            this.hood = hood;
            this.shooter = shooter;
            this.superstructure = superstructure;
        }

        public void setTarget(Pose2d robotPose, Translation2d target) {
            Rotation2d heading = target.minus(robotPose.getTranslation()).getAngle().minus(robotPose.getRotation());
            targetHeadingRad[0] = heading.getRadians();
        }

        public void forceTrack() {
            superstructure.stateMachine().force(SuperState.TRACK);
        }

        public void forceFire() {
            superstructure.stateMachine().force(SuperState.FIRE);
        }

        public void forceStowed() {
            superstructure.stateMachine().force(SuperState.STOWED);
        }
    }

    public static Handle create(String canbus) {
        double[] targetHeadingRad = new double[1];

        @SuppressWarnings("unchecked")
        StatefulMechanism<TurretState> turret = (StatefulMechanism<TurretState>) MechanismDefinitions.build(
                Mechanisms.stateful("turret", TurretState.NEUTRAL)
                        .identity(identity -> identity
                                .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.RADIANS)
                                .travelRange(Math.toRadians(-170.0), Math.toRadians(170.0)))
                        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X60, 1))
                        .encoders(encoders -> encoders.add("main", AthenaEncoder.CANCODER, 1))
                        .behavior(behavior -> behavior
                                .control(control -> control.pid("main", pid -> pid
                                        .output(OutputType.PERCENT)
                                        .kp(2.0)
                                        .ki(0.0)
                                        .kd(0.1)))
                                .automation(automation -> automation.onStatePeriodic(
                                        ctx -> ctx.mechanism().setpoint(targetHeadingRad[0]),
                                        TurretState.TARGET)))
                        .definition());

        @SuppressWarnings("unchecked")
        StatefulMechanism<HoodState> hood = (StatefulMechanism<HoodState>) MechanismDefinitions.build(
                Mechanisms.stateful("hood", HoodState.STOW)
                        .identity(identity -> identity
                                .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.RADIANS)
                                .travelRange(Math.toRadians(0.0), Math.toRadians(60.0)))
                        .motors(motors -> motors.add(AthenaMotor.NEO_V1, 2))
                        .encoders(encoders -> encoders.add("main", AthenaEncoder.SPARK_MAX, 2))
                        .behavior(behavior -> behavior.control(control -> control.pid("main", pid -> pid
                                .output(OutputType.PERCENT)
                                .kp(3.0)
                                .ki(0.0)
                                .kd(0.2))))
                        .definition());

        @SuppressWarnings("unchecked")
        StatefulMechanism<ShooterState> shooter = (StatefulMechanism<ShooterState>) MechanismDefinitions.build(
                Mechanisms.stateful("shooter", ShooterState.OFF)
                        .identity(identity -> identity.positionDomain(PositionDomainKind.VELOCITY, PositionUnit.RADIANS))
                        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X44, 3))
                        .encoders(encoders -> encoders.add("main", AthenaEncoder.CANCODER, 3))
                        .behavior(behavior -> behavior.control(control -> control
                                .feedforward("mainFF", ff -> ff
                                        .simple()
                                        .ks(0.15)
                                        .kv(0.12)
                                        .ka(0.0))
                                .pid("mainPid", pid -> pid
                                        .output(OutputType.PERCENT)
                                        .inputSource(ca.frc6390.athena.mechanisms.MechanismInputSource.Velocity)
                                        .kp(0.1)
                                        .ki(0.0)
                                        .kd(0.0))))
                        .definition());

        @SuppressWarnings("unchecked")
        SuperstructureMechanism<SuperState, SuperSetpoint> superstructure =
                (SuperstructureMechanism<SuperState, SuperSetpoint>) SuperstructureDefinitions.build(
                        Superstructures.<SuperState, SuperSetpoint>stateful("turret-super", SuperState.STOWED)
                                .mechanisms(m -> m
                                        .existing(turret, SuperSetpoint::turret)
                                        .existing(hood, SuperSetpoint::hood)
                                        .existing(shooter, SuperSetpoint::shooter))
                                .definition());

        return new Handle(targetHeadingRad, turret, hood, shooter, superstructure);
    }
}
