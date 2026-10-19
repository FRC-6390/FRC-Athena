package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Volts;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.motion.MotionPlanners;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.model.Gravity;
import frc.robot.Constants;

public final class Turret implements Mechanism {
    /** 100:1 turret reduction from the Kraken X44 pinion to the ring gear. */
    private static final GearRatio GEARING = GearRatio.reduction(100.0, 1.0);

    private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 16).brake().currentLimit(25);
    private final EncoderDevice angle = motor.encoder()
            .gearing(GEARING)
            .conversion(360.0)
            .units(EncoderUnit.DEGREES);
    private final ImuDevice chassisImu = Constants.RIO.imu(ImuKinds.PIGEON_2, 17);
    private final Range travel = Range.degrees(-270.0, 270.0);

    /**
     * Simulated plant: the geared X44 with about 0.5 kg m^2 of turret inertia. The range is in mechanism
     * rotations, so the +/-270 degree travel is +/-0.75 rotations.
     */
    private final SimModel simulation = SimModel.motor(motor)
            .gearRatio(GEARING)
            .momentOfInertia(0.5)
            .range(Range.rotations(-0.75, 0.75));

    private final ControlBinding fieldHeading = Controls.position(motor)
            .feedback(angle)
            .dependency(chassisImu)
            .loop(ControlLoops.targetTransform(binding -> context ->
                    ControlOutput.position(context.target() - chassisImu.angleDegrees())))
            .pid(0.144, 0.0, 0.006)
            .constraints(Constraints.range(travel), Constraints.motion(180.0, 540.0))
            .planner(MotionPlanners.boundedAngular(360.0));

    public final Action forward = fieldHeading.position(0.0).untilWithin(1.5);
    public final Action left = fieldHeading.position(90.0).untilWithin(1.5);
    public final Action rear = fieldHeading.position(180.0).untilWithin(1.5);
    public final Action neutral = fieldHeading.neutral();
    /** Identifies the turret: a rotary mechanism with no gravity load. Routine, quasistatic and dynamic actions. */
    public final Characterization characterization = fieldHeading.characterization()
            .gravity(Gravity.NONE)
            .maxVoltage(Volts.of(4.0));
}
