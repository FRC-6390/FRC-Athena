package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Celsius;

import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Sag;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;


public final class Feeder implements Mechanism {
    private static final PowerProfile FEEDER = PowerProfile.of()
            .sustained(Amps.of(30))
            .burst(Amps.of(40), Seconds.of(0.3))
            .share(1.5)
            .onSag(Sag.shed(0.5).below(Volts.of(8.0)));
    /** Old NEOs: the current limit slides from full at 60 C to 30 percent at 90 C (it never brakes or locks out). */
    private static final Thermal NEO_DERATE = Thermal.derate(Celsius.of(60.0), Celsius.of(90.0), 0.3);

    // Devices
//     private final MotorDevice rightLeader = Constants.RIO.motor(MotorKinds.NEO, 22).currentLimit(100);
    private final MotorDevice rightFollower = Constants.RIO.motor(MotorKinds.NEO, 23)
            .currentLimit(Amps.of(40)).power(FEEDER).thermal(NEO_DERATE);
    private final MotorDevice leftFollower = Constants.RIO.motor(MotorKinds.NEO, 24)
            .inverted()
            .follow(rightFollower).currentLimit(Amps.of(40)).power(FEEDER).thermal(NEO_DERATE);
    private final MotorDevice leftRearFollower = Constants.RIO.motor(MotorKinds.NEO, 25)
            .inverted()
            .follow(rightFollower).currentLimit(Amps.of(40)).power(FEEDER).thermal(NEO_DERATE);

    // Bindings and controls
    @Telemetry(writable = true, min = 0.0, max = 1.0)
    private double outputPercent = 0.25;

    // Actions
    public final Action feed = rightFollower.percent(() -> -outputPercent);
    public final Action reverse = rightFollower.percent(() -> outputPercent);
    public final Action neutral = rightFollower.neutral();
}
