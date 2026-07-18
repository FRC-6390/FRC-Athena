package ca.frc6390.athena.robot;

import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.mechanism.core.MechanismTraceSnapshot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** Query helpers for explaining the latest immutable runtime cycle. */
public final class RuntimeInspection {
    private final RuntimeCycleSnapshot snapshot;

    RuntimeInspection(RuntimeCycleSnapshot snapshot) {
        this.snapshot = snapshot;
    }

    public RuntimeCycleSnapshot snapshot() {
        return snapshot;
    }

    public Optional<MechanismTraceSnapshot> mechanism(String path) {
        return snapshot.mechanisms().stream().filter(value -> value.mechanism().equals(path)).findFirst();
    }

    public Optional<MechanismTraceSnapshot.Motor> motor(String name) {
        return snapshot.mechanisms().stream()
                .flatMap(mechanism -> mechanism.motors().stream())
                .filter(value -> value.name().equals(name))
                .findFirst();
    }

    /** Explains ownership, control math, command, feedback, and health for one motor. */
    public Optional<MotorInspection> explainMotor(String name) {
        for (MechanismTraceSnapshot mechanism : snapshot.mechanisms()) {
            Optional<MechanismTraceSnapshot.Motor> motor = mechanism.motors().stream()
                    .filter(value -> value.name().equals(name))
                    .findFirst();
            if (motor.isEmpty()) continue;
            MechanismTraceSnapshot.ActionCandidate owner = mechanism.candidates().stream()
                    .filter(MechanismTraceSnapshot.ActionCandidate::selected)
                    .filter(candidate -> candidate.motors().contains(name))
                    .findFirst()
                    .orElse(null);
            MechanismTraceSnapshot.Control control = mechanism.controls().stream()
                    .filter(value -> value.name().equals(name))
                    .findFirst()
                    .orElse(null);
            HardwareCycleSnapshot.MotorInput input = snapshot.hardware().motors().values().stream()
                    .filter(value -> value.name().equals(name))
                    .findFirst()
                    .orElse(null);
            return Optional.of(new MotorInspection(mechanism.mechanism(), motor.get(), owner, control, input));
        }
        return Optional.empty();
    }

    public Optional<MechanismTraceSnapshot.Control> control(String name) {
        return snapshot.mechanisms().stream()
                .flatMap(mechanism -> mechanism.controls().stream())
                .filter(value -> value.name().equals(name))
                .findFirst();
    }

    public Optional<MechanismTraceSnapshot.Hook> hook(String name) {
        return snapshot.mechanisms().stream()
                .flatMap(mechanism -> mechanism.hooks().stream())
                .filter(value -> value.name().equals(name))
                .findFirst();
    }

    /** Returns disconnected hardware with the backend failure that caused it. */
    public List<DeviceFault> faults() {
        List<DeviceFault> faults = new ArrayList<>();
        HardwareCycleSnapshot hardware = snapshot.hardware();
        hardware.motors().forEach((identity, input) -> {
            if (!input.connected()) faults.add(new DeviceFault(identity.key(), input.name(), input.failure()));
        });
        hardware.encoders().forEach((identity, input) -> {
            if (!input.connected()) faults.add(new DeviceFault(identity.key(), input.name(), input.failure()));
        });
        hardware.imus().forEach((identity, input) -> {
            if (!input.connected()) faults.add(new DeviceFault(identity.key(), input.name(), input.failure()));
        });
        return List.copyOf(faults);
    }

    public record DeviceFault(String identity, String name, String reason) { }

    public record MotorInspection(
            String mechanism,
            MechanismTraceSnapshot.Motor motor,
            MechanismTraceSnapshot.ActionCandidate owner,
            MechanismTraceSnapshot.Control control,
            HardwareCycleSnapshot.MotorInput input) { }
}
