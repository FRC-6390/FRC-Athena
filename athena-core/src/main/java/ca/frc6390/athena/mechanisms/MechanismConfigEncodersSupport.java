package ca.frc6390.athena.mechanisms;

@SuppressWarnings("unchecked")
abstract class MechanismConfigEncodersSupport<T extends Mechanism> {

    protected MechanismConfig<T> mechanismConfigSelf() {
        return (MechanismConfig<T>) this;
    }

    protected MechanismConfig.EncodersSection<T> newEncodersSection() {
        return new MechanismConfig.EncodersSection<>(mechanismConfigSelf());
    }

    public final MechanismConfig<T> encoders(MechanismEncodersSection section) {
        if (section != null) {
            section.apply(newEncodersSection());
        }
        return mechanismConfigSelf();
    }
}
