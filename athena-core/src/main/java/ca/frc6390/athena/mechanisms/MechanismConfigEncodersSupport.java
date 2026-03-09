package ca.frc6390.athena.mechanisms;

abstract class MechanismConfigEncodersSupport<T extends Mechanism> {

    protected abstract MechanismConfig<T> mechanismConfigSelf();

    protected abstract MechanismConfig.EncodersSection<T> newEncodersSection();

    public final MechanismConfig<T> encoders(MechanismEncodersSection section) {
        if (section != null) {
            section.apply(newEncodersSection());
        }
        return mechanismConfigSelf();
    }
}
