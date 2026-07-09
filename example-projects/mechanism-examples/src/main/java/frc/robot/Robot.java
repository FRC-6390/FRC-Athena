package frc.robot;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.DigitalInputs;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import edu.wpi.first.math.util.Units;

public final class Robot extends AthenaRobot {
    public final Shooter shooter = new Shooter();
    public final Arm arm = new Arm();
    public final Intake intake = new Intake();
    public final Elevator elevator = new Elevator();
    public final Superstructure superstructure = new Superstructure(shooter, arm, intake, elevator);

    @SuppressWarnings("unused")
    public final HookBinding runExample = Events.teleopPeriodic().whileActive(superstructure.scoreHigh::request);

    @Override
    protected void configure() {
        register(superstructure);
    }

    public static final class Shooter implements Mechanism {
        private final MotorDevice leader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1);
        private final MotorDevice follower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2).follow(leader).inverted();
        private final EncoderDevice velocity = leader.encoder();
        @SuppressWarnings("unused")
        private final SimModel simulation = SimModels.flywheel(leader, follower)
                .encoder(velocity)
                .momentOfInertia(0.006);
        private final ControlBinding wheel = Controls.velocity(leader)
                .feedback(velocity)
                .slot(0)
                .pid(PidGains.of(0.08, 0.0, 0.0).tolerance(2.0))
                .feedforward(FeedforwardGains.simple(0.2, 0.12, 0.0));

        public final Action stop = leader.percent(0.0);
        public final Action idle = wheel.velocity(35.0);
        public final Action podium = wheel.velocity(78.0);
        public final Action reverse = wheel.velocity(-25.0);
    }

    public static final class Arm implements Mechanism {
        private final MotorDevice shoulderMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 3).currentLimit(35);
        private final MotorDevice wristMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 4).currentLimit(20);
        private final EncoderDevice shoulderEncoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 3);
        private final EncoderDevice wristEncoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 4);
        private final DigitalInputDevice homeSwitch = DigitalInputs.rio(0).inverted();
        private final Range shoulderTravel = Range.degrees(-25.0, 95.0);
        private final Range wristTravel = Range.degrees(-70.0, 80.0);
        @SuppressWarnings("unused")
        private final SimModel shoulderSim = SimModels.arm(shoulderMotor)
                .encoder(shoulderEncoder)
                .range(shoulderTravel)
                .lengthMeters(Units.inchesToMeters(22.0))
                .gearRatio(GearRatio.reduction(60.0, 1.0));
        @SuppressWarnings("unused")
        private final SimModel wristSim = SimModels.arm(wristMotor)
                .encoder(wristEncoder)
                .range(wristTravel)
                .lengthMeters(Units.inchesToMeters(12.0))
                .gearRatio(GearRatio.reduction(30.0, 1.0));

        private final ControlBinding shoulder = Controls.position(shoulderMotor)
                .feedback(shoulderEncoder)
                .pid(0.08, 0.0, 0.002);
        private final ControlBinding wrist = Controls.position(wristMotor)
                .feedback(wristEncoder)
                .pid(0.06, 0.0, 0.001);

        public final Action stow = Actions.sequence()
                .until(homeSwitch::active, shoulderMotor.percent(() -> homeSwitch.active() ? 0.0 : -0.15))
                .then(Actions.parallel(shoulder.position(0.0).clamp(shoulderTravel), wrist.position(0.0).clamp(wristTravel)));
        public final Action floorPickup = pose(-18.0, -45.0);
        public final Action ampScore = pose(65.0, 40.0);
        public final Action trapScore = pose(92.0, 70.0);

        private Action pose(double shoulderDegrees, double wristDegrees) {
            return Actions.parallel(
                    shoulder.position(shoulderDegrees).clamp(shoulderTravel),
                    wrist.position(wristDegrees).clamp(wristTravel));
        }
    }

    public static final class Intake implements Mechanism {
        private final MotorDevice roller = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 5);

        public final Action stop = roller.percent(0.0);
        public final Action intake = roller.percent(0.8);
        public final Action hold = roller.voltage(2.0);
        public final Action eject = roller.percent(-0.6);
    }

    public static final class Elevator implements Mechanism {
        private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 6).currentLimit(40);
        private final EncoderDevice height = motor.encoder();
        private final Range travel = Range.of(0.0, 1.4);
        @SuppressWarnings("unused")
        private final SimModel simulation = SimModels.elevator(motor).encoder(height).range(travel);
        private final ControlBinding lift = Controls.position(motor).feedback(height).pid(0.5, 0.0, 0.0);

        public final Action home = lift.position(0.0).clamp(travel);
        public final Action low = lift.position(0.35).clamp(travel);
        public final Action high = lift.position(1.25).clamp(travel);
    }

    public static final class Superstructure implements Mechanism {
        private final Shooter shooter;
        private final Arm arm;
        private final Intake intake;
        private final Elevator elevator;

        public final Action home;
        public final Action collectFloor;
        public final Action scoreHigh;
        public final Action ejectAll;

        private Superstructure(Shooter shooter, Arm arm, Intake intake, Elevator elevator) {
            this.shooter = shooter;
            this.arm = arm;
            this.intake = intake;
            this.elevator = elevator;
            home = Actions.set().set(shooter, shooter.stop).set(arm, arm.stow).set(intake, intake.stop).set(elevator, elevator.home);
            collectFloor = Actions.set().set(shooter, shooter.stop).set(arm, arm.floorPickup).set(intake, intake.intake).set(elevator, elevator.low);
            scoreHigh = Actions.sequence()
                    .forTime(0.75, Actions.set().set(shooter, shooter.podium).set(arm, arm.trapScore).set(elevator, elevator.high))
                    .then(Actions.set().set(shooter, shooter.podium).set(arm, arm.trapScore).set(intake, intake.eject).set(elevator, elevator.high));
            ejectAll = Actions.set().set(shooter, shooter.reverse).set(arm, arm.ampScore).set(intake, intake.eject).set(elevator, elevator.low);
        }
    }
}
