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
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;


public final class IntakeRoller implements Mechanism {
    private static final PowerProfile INTAKE = PowerProfile.of()
            .sustained(Amps.of(30))
            .burst(Amps.of(40), Seconds.of(0.3))
            .share(1.0)
            .onSag(Sag.shed(0.5).below(Volts.of(8.0)));
    /** Old NEOs: the current limit slides from full at 60 C to 30 percent at 90 C (it never brakes or locks out). */
    private static final Thermal NEO_DERATE = Thermal.derate(Celsius.of(60.0), Celsius.of(90.0), 0.3);

    // Devices
    private final MotorDevice leader = Constants.RIO.motor(MotorKinds.NEO, 41).currentLimit(Amps.of(40)).neutralMode(MotorNeutralMode.COAST)
            .power(INTAKE).thermal(NEO_DERATE);
    private final MotorDevice follower = Constants.RIO.motor(MotorKinds.NEO, 42).neutralMode(MotorNeutralMode.COAST)
            
            .follow(leader).currentLimit(Amps.of(40)).inverted()
            .power(INTAKE).thermal(NEO_DERATE);

    // Bindings and controls
    @Telemetry(writable = true, min = 0.0, max = 1.0)
    private double outputPercent = 0.5;

    // Actions
    public final Action intake = leader.percent(() -> -outputPercent);
    public final Action reverse = leader.percent(() -> outputPercent);
    public final Action neutral = leader.neutral();
    private final Action clearJam = Actions.sequence()
            .forTime(Seconds.of(0.1), reverse)
            .forTime(Seconds.of(0.05), neutral);
    public final HookBinding clearJamOnStall = Events.when(
                    leader.stall().forTime(Seconds.of(0.07)).rearmAfter(Seconds.of(0.1)))
            .rising()
            .onStart(clearJam);

        
}
