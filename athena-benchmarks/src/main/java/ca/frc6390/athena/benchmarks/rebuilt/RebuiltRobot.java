// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import static edu.wpi.first.units.Units.Degrees;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.wpilib.geometry.AllianceTransform;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;





/** Copy of the Rebuilt off-season robot (without Choreo autos) used to profile the full Athena robot loop. */
public final class RebuiltRobot extends AthenaRobot {
    /** Runs one WPILib TimedRobot iteration (driver station refresh, mode transitions and all callbacks). */
    public void cycle() {
        loopFunc();
    }

    private static final double INTAKE_STOW_SECONDS = 0.5;

    public final Drivetrain drivetrain = new Drivetrain();
    public final Localization localization = new Localization(drivetrain);
    public final Feeder feeder = new Feeder();
    public final Hood hood = new Hood(localization);
    public final IntakeArm intakeArm = new IntakeArm();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public final Shooter shooter = new Shooter(localization);

    public final Action faceHub = drivetrain.aimAtHub(localization::pose);
    public final Action intaking = Actions.sequence()
        //      .forTime(INTAKE_STOW_SECONDS, intakeArm.stow)
            .start(intakeRoller.intake)
            .then(intakeArm.deploy);

    public final Action aimAtHub = Actions.parallel(
            faceHub,
            hood.aim);
    // faceHub never completes, so it runs alongside the whole shot (spin-up and feed) instead of
    // holding the spin-up stage open until its timeout.
    public final Action shoot = Actions.parallel(
            faceHub,
            Actions.sequence()
                    .run(
                            Actions.parallel(
                                    // settled(): ready only once the value has stayed in the window,
                                    // not the first cycle it swings through it.
                                    shooter.fire.untilWithin(RotationsPerSecond.of(2.0)).settled(Seconds.of(0.1)),
                                    hood.aim.untilWithin(Degrees.of(1.0)).settled(Seconds.of(0.1))).timeout(Seconds.of(1.0))
                            )
                    .then(Actions.parallel(
                            shooter.fire,
                            intakeArm.stow,
                            intakeRoller.intake,
                            feeder.feed)));
    public final Action collect = Actions.parallel(
            intaking);

              public final Action repeat = Actions.sequence().forTime(Seconds.of(1.0), intaking).forTime(Seconds.of(0.5), intakeArm.stow);

              public final Action again = Actions.sequence(
          
              ).then(repeat).then(repeat).then(collect);

    public final Action stopAll = Actions.parallel(
            intakeArm.neutral,
            intakeRoller.neutral,
            shooter.neutral,
            hood.neutral,
            feeder.neutral);
    public final Action homeAll = Actions.parallel(
            intakeArm.home,
            hood.home);
    public final Action reverseAll = Actions.parallel(
            feeder.reverse,
            shooter.reverse,
            intakeRoller.reverse);

    public  final Action manualShoot = Actions.parallel(
            intakeArm.cycle,    
            shooter.fire,
            hood.aim,
            feeder.feed
    );

     public  final Action stow = Actions.parallel(
            intakeArm.stow,    
            intakeRoller.neutral
    );



    /**
     * Saves the current shot (distance, wheel speed and hood angle, trims included) into both shot
     * tables, then clears the trims. Field tuning: drive to a spot, trim with the operator POV and
     * bumpers until the shot goes in, press record. Points are kept across reboots and deploys.
     */
    public final Action recordShot = Actions.sequence()
            .run(Actions.parallel(shooter.recordShot, hood.recordShot))
            .then(Actions.parallel(shooter.speedReset, hood.mHoodReset));

    /** Battery floor and start staggering for the power profiles declared on each mechanism. */
    public final PowerPolicy power = PowerPolicy.defaults()
            .floor(Volts.of(7.5))
            .staggerStarts(Seconds.of(0.1));

    public final Controls controls = new Controls(this);

    // Field geometry is written once in blue coordinates; ALLIANCE mirrors it across the centerline
    // for red (the same red reset pose as before: 13.04, 4.0, facing 0 degrees).
    private static final AllianceTransform ALLIANCE = AllianceTransform.mirrorX(Constants.FIELD_LENGTH);
    private static final Pose2d RESET_POSE_BLUE = new Pose2d(3.5, 4.0, Rotation2d.k180deg);

    // The alliance is checked when the action runs (button press), not when Controls is built.
    public Action resetPose(){
        return localization.reset(() -> ALLIANCE.pose(RESET_POSE_BLUE));
    }
}
