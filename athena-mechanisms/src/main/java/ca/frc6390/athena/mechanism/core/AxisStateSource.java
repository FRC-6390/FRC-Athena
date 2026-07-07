package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionContext;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.RuntimeEncoder;
import java.util.Objects;
import java.util.function.Function;

/**
 * Runtime lookup for live axis state used by rules.
 */
public interface AxisStateSource {
    AxisState axis(AxisRef axis);

    static AxisStateSource empty() {
        return axis -> new AxisState() {
        };
    }

    static AxisStateSource of(Function<AxisRef, AxisState> lookup) {
        Objects.requireNonNull(lookup, "lookup");
        return axis -> {
            AxisState state = lookup.apply(axis);
            return state == null ? new AxisState() {
            } : state;
        };
    }

    static AxisStateSource from(ActionContext context) {
        Objects.requireNonNull(context, "context");
        return axis -> {
            if (axis == null || axis.encoders().isEmpty()) {
                return new AxisState() {
                };
            }
            EncoderRef ref = axis.encoders().get(0);
            RuntimeEncoder encoder = context.encoder(ref);
            return new AxisState() {
                @Override
                public double position() {
                    return encoder.position();
                }

                @Override
                public double velocity() {
                    return encoder.velocity();
                }
            };
        };
    }
}
