package ca.frc6390.athena.mechanism.ref;

import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.GearRatioRef;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopRef;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import java.util.List;
import java.util.Objects;

/**
 * Chinese-remainder-theorem feedback declaration.
 */
public record CrtRef(EncoderRef coarse, EncoderRef fine, GearRatioRef ratio) implements ControlLoopRef {
    public CrtRef {
        Objects.requireNonNull(coarse, "coarse");
        Objects.requireNonNull(fine, "fine");
        Objects.requireNonNull(ratio, "ratio");
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime();
    }

    @Override
    public List<Object> refs() {
        return List.of(coarse, fine, ratio);
    }

    private static final class Runtime implements ControlLoopRuntime {
        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            return ControlOutput.position(context.target());
        }
    }
}
