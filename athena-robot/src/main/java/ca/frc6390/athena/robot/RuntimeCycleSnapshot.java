package ca.frc6390.athena.robot;

import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismTraceSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import java.util.List;

/** Immutable view of all runtime data used and produced during one robot cycle. */
public record RuntimeCycleSnapshot(
        long sequence,
        double timestampSeconds,
        double dtSeconds,
        LifecycleMode mode,
        LifecyclePhase phase,
        boolean enabled,
        boolean autonomous,
        boolean simulation,
        HardwareCycleSnapshot hardware,
        List<DigitalInput> digitalInputs,
        List<MechanismTraceSnapshot> mechanisms,
        List<LocalizationInput> localizations) {
    public RuntimeCycleSnapshot {
        mode = mode == null ? LifecycleMode.ROBOT : mode;
        phase = phase == null ? LifecyclePhase.PERIODIC : phase;
        digitalInputs = digitalInputs == null ? List.of() : List.copyOf(digitalInputs);
        mechanisms = mechanisms == null ? List.of() : List.copyOf(mechanisms);
        localizations = localizations == null ? List.of() : List.copyOf(localizations);
    }

    public record DigitalInput(String name, int channel, boolean raw, boolean active) {
        public DigitalInput { name = name == null ? "" : name; }
    }

    public record LocalizationInput(int index, String type, List<Measurement> measurements) {
        public LocalizationInput {
            type = type == null ? "" : type;
            measurements = measurements == null ? List.of() : List.copyOf(measurements);
        }
    }
}
