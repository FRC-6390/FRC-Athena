package ca.frc6390.athena.hardware.encoder;

import java.util.List;

/**
 * Averages multiple encoder inputs of the same signal kind.
 */
public final class AverageEncoder extends AbstractDerivedEncoder {
    private final List<DerivedEncoderInput> inputs;

    public AverageEncoder(
            EncoderConfig config,
            List<DerivedEncoderInput> inputs,
            EncoderSignalType outputSignal) {
        super(
                config,
                outputSignal == EncoderSignalType.VELOCITY ? OutputKind.VELOCITY : OutputKind.POSITION,
                outputSignal == EncoderSignalType.ABSOLUTE);
        this.inputs = List.copyOf(inputs);
    }

    @Override
    public Encoder update() {
        double sum = 0.0;
        int count = 0;
        for (DerivedEncoderInput input : inputs) {
            sum += input.read(true);
            count++;
        }
        cachedValue = count > 0 ? sum / count : 0.0;
        return this;
    }
}
