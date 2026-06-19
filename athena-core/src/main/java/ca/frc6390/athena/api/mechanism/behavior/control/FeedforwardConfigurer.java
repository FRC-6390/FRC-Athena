package ca.frc6390.athena.api.mechanism.behavior.control;

@FunctionalInterface
public interface FeedforwardConfigurer {
    MechanismFeedforward apply(MechanismFeedforward feedforward);
}
