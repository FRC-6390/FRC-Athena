package ca.frc6390.athena.hardware.encoder;

import java.util.List;

/**
 * Subtracts the second encoder input from the first.
 */
public final class DifferenceEncoder extends AbstractDerivedEncoder {
    private final DerivedEncoderInput left;
    private final DerivedEncoderInput right;

    public DifferenceEncoder(
            EncoderConfig config,
            List<DerivedEncoderInput> inputs,
            EncoderSignalType outputSignal) {
        super(
                config,
                outputSignal == EncoderSignalType.VELOCITY ? OutputKind.VELOCITY : OutputKind.POSITION,
                outputSignal == EncoderSignalType.ABSOLUTE);
        if (inputs == null || inputs.size() != 2) {
            throw new IllegalArgumentException("difference encoder requires exactly 2 inputs");
        }
        this.left = inputs.get(0);
        this.right = inputs.get(1);
    }

    @Override
    public Encoder update() {
        cachedValue = left.read(true) - right.read(true);
        return this;
    }
}
