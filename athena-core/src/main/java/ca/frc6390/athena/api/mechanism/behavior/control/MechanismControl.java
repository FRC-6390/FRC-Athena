package ca.frc6390.athena.api.mechanism.behavior.control;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.api.mechanism.definition.MechanismCustomControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismBangBangControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismPidControllerDefinition;

public final class MechanismControl {
    private final List<MechanismLoopDefinition> definitions = new ArrayList<>();

    private MechanismControl() {
    }

    public static MechanismControl create() {
        return new MechanismControl();
    }

    public static MechanismControl from(List<MechanismLoopDefinition> loops) {
        MechanismControl control = create();
        loops.forEach(control.definitions::add);
        return control;
    }

    public MechanismControl pid(String name, PidConfigurer configurer) {
        MechanismPid pid = MechanismPid.create(name);
        if (configurer != null) {
            configurer.apply(pid);
        }
        return add(pid);
    }

    public MechanismControl feedforward(String name, FeedforwardConfigurer configurer) {
        MechanismFeedforward feedforward = MechanismFeedforward.create(name);
        if (configurer != null) {
            configurer.apply(feedforward);
        }
        return add(feedforward);
    }

    public MechanismControl bangBang(String name, BangBangConfigurer configurer) {
        MechanismBangBang bangBang = MechanismBangBang.create(name);
        if (configurer != null) {
            configurer.apply(bangBang);
        }
        return add(bangBang);
    }

    public MechanismControl customLoop(String name, ControlLoopConfigurer configurer) {
        MechanismControlLoop loop = MechanismControlLoop.create(name);
        if (configurer != null) {
            configurer.apply(loop);
        }
        return add(loop);
    }

    public MechanismControl add(MechanismPid pid) {
        definitions.add(pid.definition());
        return this;
    }

    public MechanismControl add(MechanismFeedforward feedforward) {
        definitions.add(feedforward.definition());
        return this;
    }

    public MechanismControl add(MechanismBangBang bangBang) {
        definitions.add(bangBang.definition());
        return this;
    }

    public MechanismControl add(MechanismControlLoop loop) {
        definitions.add(loop.definition());
        return this;
    }

    public MechanismControl merge(MechanismControl other) {
        if (other != null) {
            definitions.addAll(other.definitions);
        }
        return this;
    }

    public List<MechanismLoopDefinition> definitions() {
        return List.copyOf(definitions);
    }

    public List<MechanismPid> pids() {
        return definitions.stream()
            .filter(definition -> definition.controller() instanceof MechanismPidControllerDefinition)
            .map(MechanismPid::from)
            .toList();
    }

    public List<MechanismFeedforward> feedforwards() {
        return definitions.stream()
            .filter(definition -> definition.controller() instanceof MechanismFeedforwardControllerDefinition)
            .map(MechanismFeedforward::from)
            .toList();
    }

    public List<MechanismBangBang> bangBangs() {
        return definitions.stream()
            .filter(definition -> definition.controller() instanceof MechanismBangBangControllerDefinition)
            .map(MechanismBangBang::from)
            .toList();
    }

    public List<MechanismControlLoop> customLoops() {
        return definitions.stream()
            .filter(definition -> definition.controller() instanceof MechanismCustomControllerDefinition)
            .map(MechanismControlLoop::from)
            .toList();
    }
}
