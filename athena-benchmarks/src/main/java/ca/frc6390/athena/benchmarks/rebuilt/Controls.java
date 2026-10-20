package ca.frc6390.athena.benchmarks.rebuilt;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.controls.AxisCurves;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.DoubleSupplier;

public final class Controls implements Mechanism {
        public final Gamepad driver = Controllers.xbox(0);
        public final Gamepad operator = Controllers.xbox(1);
    public final AxisCurves.SuperRateCurve forwardCurve = AxisCurves.superRate()
            .rcRate(1.0)
            .expo(0.25)
            .superRate(0.65);

    private final DoubleSupplier forward = driver.leftY()
            .deadband(0.02)
            .inverted()
            .curve(forwardCurve)
            .squared()
            .slew(4.0, 6.0)
            .toSupplier();
    private final DoubleSupplier strafe = driver.leftX()
            .deadband(0.02)
            .inverted()
            .curve(forwardCurve)
            .squared()
            .slew(4.0, 6.0)
            .toSupplier();
    private final DoubleSupplier rotation = driver.rightX()
            .deadband(0.02)
            .inverted()
            .squared()
            .toSupplier();

    private boolean fieldOriented = true;
     private final Action driveAction;
     public final HookBinding driverControl;

    public Controls(RebuiltRobot robot) {

        driveAction = robot.drivetrain.drive(forward, strafe, rotation, () -> fieldOriented);
         driverControl = Events.when(DriverStation::isEnabled)
                 .active()
                 .whileActive(driveAction);

        //Game Controls
        driver.leftTrigger(0.15).onPress(robot.intaking);
        driver.rightTrigger(0.15).whileHeld(robot.shoot).onRelease(robot.intaking);


        //Odometry/POSE reset
          driver.x().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.resetPose()); 
        

        driver.start().pressed().onlyIf(DriverStation::isTeleopEnabled)
                .onTrue(robot.drivetrain.resetDriverHeading);


        // POV Buttons
        driver.povDown().onPress(robot.stopAll);
        driver.povLeft().onPress(robot.reverseAll);
        operator.povRight().onTrue(robot.shooter.speedDown);
        
        //Manual
        driver.b().whileHeld(robot.manualShoot).onRelease(robot.collect);



       
        //Testing / Tuning

        // driver.b().whileHeld(robot.shoot).whileFalse(robot.stopAll);
        // driver.y().whileHeld(robot.aimAtHub).whileFalse(robot.stopAll);
        // driver.povUp().whileHeld(robot.intakeRoller.aintake).whileFalse(robot.intakeRoller.neutral);

        // driver.rightBumper().onPress(robot.intakeArm.deploy);
        // driver.leftBumper().onPress(robot.intakeArm.stow);
        

        // driver.rightBumper().onPress(robot.faceHub).onRelease(robot.drivetrain.stopAiming());
                // driver.leftBumper().onPress(robot.feeder.reverse).onRelease(robot.feeder.neutral);

        // driver.leftBumper().onPress(robot.intakeRoller.intake).onRelease(robot.intakeRoller.neutral);
       
        // driver.y().onPress(robot.hood.home);
        // driver.a().onPress(robot.intakeArm.deploy).onRelease(robot.intakeArm.neutral);
        // driver.b().onPress(robot.intakeArm.stow).onRelease(robot.intakeArm.neutral);
        //driver.x().onPress(robot.intakeRoller.intake).onRelease(robot.intakeRoller.neutral);
        // driver.y().onPress(robot.intakeArm.characterization.dynamicForward()).onRelease(robot.intakeArm.neutral);
        // driver.x().onPress(robot.intakeArm.characterization.dynamicReverse()).onRelease(robot.intakeArm.neutral);
        // driver.a().onPress(robot.intakeArm.characterization.quasistaticForward()).onRelease(robot.intakeArm.neutral);
        // driver.b().onPress(robot.intakeArm.characterization.quasistaticReverse()).onRelease(robot.intakeArm.neutral);


        //Operator Controller

        //Hood
        operator.rightBumper().whileHeld(robot.hood.hoodUp).onRelease(robot.hood.neutral);
        operator.leftBumper().whileHeld(robot.hood.hoodDown).onRelease(robot.hood.neutral);
        // Hold A to force the hood down; on release the hood goes back to whatever was aiming it
        // (the held action pre-empts the aim, and the aim resumes when it is released).
        operator.a().whileHeld(robot.hood.stow);

        //Shooter
        operator.povUp().onTrue(robot.shooter.speedUp);
        operator.povDown().onTrue(robot.shooter.speedDown);
        operator.rightStick().onTrue(robot.shooter.speedReset);
        operator.rightStick().onTrue(robot.hood.mHoodReset);

        operator.povLeft().whileHeld(robot.intakeRoller.reverse).onRelease(robot.intakeRoller.reverse);
        operator.leftTrigger(.1).onPress(robot.stow);

        // Shot tuning: save the current shot into the tables (not while on the field with FMS).
        operator.back().pressed().onlyIf(() -> !DriverStation.isFMSAttached())
                .onTrue(robot.recordShot);

        
        //Misc
        // operator.x().onPress(robot.homeAll);

        //Tuning
       
       operator.b().onPress(robot.shooter.shooterPercent).onRelease(robot.shooter.neutral);
         operator.b().onPress(robot.feeder.feed).onRelease(robot.feeder.neutral);
         operator.b().onPress(robot.intakeRoller.intake).onRelease(robot.intakeRoller.neutral);
         operator.b().onPress(robot.intakeArm.deploy).onRelease(robot.intakeArm.neutral);

        
    }
}
