package ca.frc6390.athena.api.mechanism.behavior.control;

import java.util.List;
import java.util.OptionalDouble;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.definition.LoopActivation;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardModel;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;
import ca.frc6390.athena.mechanisms.OutputType;

public final class MechanismFeedforward {
    private String name;
    private OutputType output = OutputType.VOLTAGE;
    private LoopMode mode = LoopMode.ENABLED;
    private List<String> states = List.of();
    private MechanismFeedforwardModel model = MechanismFeedforwardModel.SIMPLE;
    private double kS;
    private double kG;
    private double kV;
    private double kA;
    private Double tolerance;
    private MechanismSetpointSource setpointSource = MechanismSetpointSource.Setpoint;

    private MechanismFeedforward() {
    }

    public static MechanismFeedforward create() {
        return new MechanismFeedforward();
    }

    public static MechanismFeedforward create(String name) {
        return create().named(name);
    }

    public static MechanismFeedforward from(MechanismLoopDefinition definition) {
        if (!(definition.controller() instanceof MechanismFeedforwardControllerDefinition controller)) {
            throw new IllegalArgumentException("loop is not a feedforward declaration: " + definition.name());
        }
        MechanismFeedforward feedforward = create(definition.name())
            .output(definition.output())
            .schedule(definition.activation().mode(), definition.activation().states().toArray(String[]::new))
            .model(controller.model())
            .ks(controller.kS())
            .kg(controller.kG())
            .kv(controller.kV())
            .ka(controller.kA())
            .setpointSource(controller.setpointSource());
        controller.tolerance().ifPresent(feedforward::tolerance);
        return feedforward;
    }

    public MechanismFeedforward named(String name) {
        this.name = name;
        return this;
    }

    public MechanismFeedforward output(OutputType output) {
        this.output = output;
        return this;
    }

    public MechanismFeedforward schedule(LoopMode mode, String... states) {
        this.mode = mode;
        this.states = states == null ? List.of() : List.of(states);
        return this;
    }

    public MechanismFeedforward manual() {
        return schedule(LoopMode.MANUAL);
    }

    public MechanismFeedforward enabled(String... states) {
        return schedule(LoopMode.ENABLED, states);
    }

    public MechanismFeedforward model(MechanismFeedforwardModel model) {
        this.model = model != null ? model : MechanismFeedforwardModel.SIMPLE;
        return this;
    }

    public MechanismFeedforward simple() {
        return model(MechanismFeedforwardModel.SIMPLE);
    }

    public MechanismFeedforward arm() {
        return model(MechanismFeedforwardModel.ARM);
    }

    public MechanismFeedforward elevator() {
        return model(MechanismFeedforwardModel.ELEVATOR);
    }

    public MechanismFeedforward ks(double kS) {
        this.kS = kS;
        return this;
    }

    public MechanismFeedforward kg(double kG) {
        this.kG = kG;
        return this;
    }

    public MechanismFeedforward kv(double kV) {
        this.kV = kV;
        return this;
    }

    public MechanismFeedforward ka(double kA) {
        this.kA = kA;
        return this;
    }

    public MechanismFeedforward tolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public MechanismFeedforward setpointSource(MechanismSetpointSource setpointSource) {
        this.setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        return this;
    }

    public MechanismFeedforward setpointInput(String key) {
        return setpointSource(MechanismSetpointSource.input(key));
    }

    public MechanismLoopDefinition definition() {
        return new MechanismLoopDefinition(
            requiredName(),
            output,
            new LoopActivation(mode, states),
            LoopDeclarationKind.OBJECT,
            new MechanismFeedforwardControllerDefinition(
                model,
                kS,
                kG,
                kV,
                kA,
                tolerance != null ? OptionalDouble.of(tolerance) : OptionalDouble.empty(),
                setpointSource),
            getClass());
    }

    private String requiredName() {
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("feedforward loop name is required");
        }
        return name;
    }
}
