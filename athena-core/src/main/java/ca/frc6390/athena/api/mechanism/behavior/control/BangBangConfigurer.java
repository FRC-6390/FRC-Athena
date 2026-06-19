package ca.frc6390.athena.api.mechanism.behavior.control;

@FunctionalInterface
public interface BangBangConfigurer {
    MechanismBangBang apply(MechanismBangBang bangBang);
}
