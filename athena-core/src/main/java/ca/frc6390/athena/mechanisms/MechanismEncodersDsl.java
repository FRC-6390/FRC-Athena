package ca.frc6390.athena.mechanisms;

/**
 * Top-level view of the encoder-sources section so nested completion does not
 * depend on JDT understanding a generic nested member class as the outer lambda type.
 */
public interface MechanismEncodersDsl {

    MechanismEncodersDsl add(String name, MechanismEncoderSourceSection section);
}
