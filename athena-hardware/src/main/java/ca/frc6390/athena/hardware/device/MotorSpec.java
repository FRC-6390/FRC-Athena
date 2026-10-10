package ca.frc6390.athena.hardware.device;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.power.MotorThermalSpec;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

/**
 * The one table of nominal motor constants. Every consumer derives from it: the electrical model
 * ({@link MotorConstants}: Kt, Kv, R, free speed, stall and free current) used by stall detection, characterization
 * and inertia, and the thermal model ({@link MotorThermalSpec}: resistance and speed constant from the same electrical
 * values, plus thermal capacity, cooling time constant and rated winding temperature).
 *
 * <p>All values are <b>nominal</b>. They come from published stall and free-run figures and typical thermal
 * behaviour, and have not been characterized on the robot. Teams can replace them by characterization (thermal
 * run) or by {@link MotorConstants#of}. Motor kinds without an entry have no built-in constants and use
 * {@link MotorThermalSpec#GENERIC}.</p>
 *
 * @param kind motor kind this row describes
 * @param stallTorqueNm stall torque, newton-metres
 * @param stallCurrentAmps stall current, amps
 * @param freeCurrentAmps no-load current, amps
 * @param freeSpeedRpm no-load speed at 12 V, revolutions per minute
 * @param heatCapacityJoulesPerCelsius lumped winding heat capacity (nominal)
 * @param coolingSeconds first-order cooling time constant (nominal)
 * @param ambientCelsius assumed ambient temperature at boot
 * @param maxWindingCelsius rated maximum winding temperature (drives the default derate curve)
 */
public record MotorSpec(
        MotorKinds kind,
        double stallTorqueNm,
        double stallCurrentAmps,
        double freeCurrentAmps,
        double freeSpeedRpm,
        double heatCapacityJoulesPerCelsius,
        double coolingSeconds,
        double ambientCelsius,
        double maxWindingCelsius) {

    private static final Map<MotorKinds, MotorSpec> TABLE = new EnumMap<>(MotorKinds.class);

    static {
        row(MotorKinds.FALCON_500, 4.69, 257.0, 1.5, 6380.0, 260.0, 300.0, 25.0, 150.0);
        row(MotorKinds.KRAKEN_X60, 7.09, 366.0, 2.0, 5800.0, 260.0, 300.0, 25.0, 150.0);
        row(MotorKinds.KRAKEN_X44, 4.05, 275.0, 1.3, 7530.0, 170.0, 280.0, 25.0, 150.0);
        row(MotorKinds.NEO, 2.6, 105.0, 1.8, 5676.0, 180.0, 260.0, 25.0, 130.0);
        row(MotorKinds.NEO_550, 0.97, 100.0, 1.4, 11000.0, 120.0, 240.0, 25.0, 130.0);
        row(MotorKinds.NEO_VORTEX, 3.6, 211.0, 3.6, 6784.0, 200.0, 260.0, 25.0, 130.0);
        row(MotorKinds.CIM, 2.42, 133.0, 2.7, 5330.0, 350.0, 320.0, 25.0, 130.0);
        row(MotorKinds.MINI_CIM, 1.41, 89.0, 3.0, 5840.0, 180.0, 280.0, 25.0, 130.0);
        row(MotorKinds.VEX_775_PRO, 0.71, 134.0, 0.7, 18730.0, 300.0, 300.0, 25.0, 130.0);
    }

    public MotorSpec {
        if (kind == null) {
            throw new IllegalArgumentException("Motor spec needs a kind.");
        }
    }

    private static void row(MotorKinds kind, double torque, double stallCurrent, double freeCurrent, double freeRpm,
            double heatCapacity, double cooling, double ambient, double maxWinding) {
        TABLE.put(kind, new MotorSpec(kind, torque, stallCurrent, freeCurrent, freeRpm, heatCapacity, cooling,
                ambient, maxWinding));
    }

    /**
     * Returns the table row for a motor kind.
     *
     * @param kind motor kind, may be null
     * @return the row, or empty when the kind has no built-in constants
     */
    public static Optional<MotorSpec> forKind(MotorKind kind) {
        MotorKind physical = kind == null ? null : kind.motorKind();
        if (!(physical instanceof MotorKinds known)) {
            return Optional.empty();
        }
        return Optional.ofNullable(TABLE.get(known));
    }

    /** Electrical constants derived from this row (R, Kt, Kv). */
    public MotorConstants constants() {
        return MotorConstants.of(stallTorqueNm, stallCurrentAmps, freeCurrentAmps, freeSpeedRpm);
    }

    /** Thermal constants: winding resistance and speed constant come from {@link #constants()}. */
    public MotorThermalSpec thermalSpec() {
        MotorConstants constants = constants();
        return new MotorThermalSpec(constants.resistanceOhms(), constants.speedConstant(),
                heatCapacityJoulesPerCelsius, coolingSeconds, ambientCelsius, maxWindingCelsius);
    }
}
