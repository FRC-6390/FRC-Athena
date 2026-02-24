package ca.frc6390.athena.mechanisms;

public final class MechanismSimulationSection {
    private final Mechanism owner;

    MechanismSimulationSection(Mechanism owner) {
        this.owner = owner;
    }

    public MechanismSimulationSection reset() {
        owner.resetSimulationInternal();
        return this;
    }

    public MechanismSimulationSection encoderState(double position, double velocity) {
        owner.setSimulatedEncoderStateInternal(position, velocity);
        return this;
    }

    public MechanismSimulationSection clearEncoderState() {
        owner.clearSimulatedEncoderStateInternal();
        return this;
    }

    public boolean hasSimulation() {
        return owner.hasSimulationInternal();
    }

    public double updatePeriodSeconds() {
        return owner.simulationUpdatePeriodSecondsInternal();
    }
}
