package ca.frc6390.athena.mechanisms;

@FunctionalInterface
public interface MechanismEncoderSourceSection {

    MechanismEncoderSourceDsl apply(MechanismEncoderSourceDsl section);
}
