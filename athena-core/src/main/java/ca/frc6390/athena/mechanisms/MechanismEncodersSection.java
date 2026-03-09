package ca.frc6390.athena.mechanisms;

@FunctionalInterface
public interface MechanismEncodersSection {

    MechanismEncodersDsl apply(MechanismEncodersDsl section);
}
