package ca.frc6390.athena.api.mechanism.encoder;

@FunctionalInterface
public interface EncodersConfigurer {
    MechanismEncoders apply(MechanismEncoders encoders);
}
