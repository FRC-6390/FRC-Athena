package ca.frc6390.athena.mechanism.persist;

import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.model.FrictionTable;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Flattens a {@link MechanismModel} into finite doubles for the persistent store, and back.
 *
 * <p>Layout: 16 fixed scalars, then the friction table as {@code n}, {@code n} positions and {@code n} volts.
 * Unknown values (NaN backlash and latency) are stored as 0 and flagged in the mask scalar. Standard errors are not
 * stored, so a restored model reports none. Motor kind names and the motion kind are stored as strings by
 * {@link PersistentModelStorage}.</p>
 */
final class ModelCodec {
    private static final int FIXED = 16;
    private static final int MASK_BACKLASH_UNKNOWN = 1;
    private static final int MASK_LATENCY_UNKNOWN = 2;

    private ModelCodec() {
    }

    static double[] encode(MechanismModel model) {
        List<Double> positions = model.friction().positions();
        List<Double> volts = model.friction().volts();
        int n = positions.size();
        double[] out = new double[FIXED + 1 + 2 * n];
        out[0] = finite(model.staticFrictionVolts());
        out[1] = finite(model.kineticFrictionVolts());
        out[2] = finite(model.kV());
        out[3] = finite(model.kA());
        out[4] = finite(model.kG());
        out[5] = finite(model.gravityOffsetRadians());
        out[6] = finite(model.backlash());
        out[7] = finite(model.latencySeconds());
        out[8] = finite(model.payloadDelta());
        out[9] = finite(model.motorTorqueConstant());
        out[10] = finite(model.motorResistance());
        out[11] = model.motorCount();
        out[12] = finite(model.gearRatio());
        out[13] = finite(model.rSquared());
        out[14] = model.sampleCount();
        out[15] = (Double.isFinite(model.backlash()) ? 0 : MASK_BACKLASH_UNKNOWN)
                | (Double.isFinite(model.latencySeconds()) ? 0 : MASK_LATENCY_UNKNOWN);
        out[FIXED] = n;
        for (int i = 0; i < n; i++) {
            out[FIXED + 1 + i] = positions.get(i);
            out[FIXED + 1 + n + i] = volts.get(i);
        }
        return out;
    }

    /** Decodes a vector. Returns null when the vector is malformed. */
    static MechanismModel decode(double[] in, MotionKind kind, List<String> motorKinds) {
        if (in == null || in.length < FIXED + 1) {
            return null;
        }
        for (double value : in) {
            if (!Double.isFinite(value)) return null;
        }
        // These fields are stored as doubles, but represent exact integer counts. Check before casting so
        // corrupt lengths cannot overflow or silently truncate into a different, apparently valid model.
        if (!count(in[FIXED], (in.length - FIXED - 1) / 2)
                || !count(in[11], Integer.MAX_VALUE) || !count(in[14], Integer.MAX_VALUE)
                || !count(in[15], MASK_BACKLASH_UNKNOWN | MASK_LATENCY_UNKNOWN)
                || !(in[12] > 0.0)) {
            return null;
        }
        int n = (int) in[FIXED];
        if (in.length != FIXED + 1 + 2 * n) {
            return null;
        }
        int mask = (int) in[15];
        List<Double> positions = new ArrayList<>(n);
        List<Double> volts = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            positions.add(in[FIXED + 1 + i]);
            volts.add(in[FIXED + 1 + n + i]);
        }
        double backlash = (mask & MASK_BACKLASH_UNKNOWN) != 0 ? Double.NaN : in[6];
        double latency = (mask & MASK_LATENCY_UNKNOWN) != 0 ? Double.NaN : in[7];
        Map<String, Double> none = Map.of();
        return new MechanismModel(kind, in[0], in[1], in[2], in[3], in[4], in[5],
                new FrictionTable(positions, volts), backlash, latency, in[8], in[9], in[10],
                (int) in[11], motorKinds, in[12], none, in[13], (int) in[14]);
    }

    private static boolean count(double value, int maximum) {
        return value >= 0.0 && value <= maximum && value == Math.rint(value);
    }

    private static double finite(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
