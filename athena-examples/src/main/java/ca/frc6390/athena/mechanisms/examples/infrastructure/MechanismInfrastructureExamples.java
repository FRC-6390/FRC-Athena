package ca.frc6390.athena.mechanisms.examples;

import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.MechanismPidAutotuners;
import ca.frc6390.athena.mechanisms.OutputConversions;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.RegisterableMechanism;
import ca.frc6390.athena.mechanisms.RegisterableMechanismFactory;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Examples covering mechanism infrastructure APIs (sections, registration, autotune, visualization).
 */
public final class MechanismInfrastructureExamples {
    private MechanismInfrastructureExamples() {}

    public static Mechanism createConfiguredMechanism(String name) {
        String mechanismName = Objects.requireNonNull(name, "name").trim();
        if (mechanismName.isEmpty()) {
            throw new IllegalArgumentException("name must not be blank");
        }
        Mechanism mechanism = MechanismConfig.generic(mechanismName).build();
        configureSafeSysId(mechanism, 0.35, 2.0, 5.0, 4.0);
        configureNetworkTables(mechanism, "Robot/Mechanisms/" + mechanismName, 0.05, "safe-bindings");
        return mechanism;
    }

    public static void configureSafeSysId(
            Mechanism mechanism,
            double rampRateVoltsPerSecond,
            double stepVoltage,
            double timeoutSeconds,
            double voltageLimit) {
        Objects.requireNonNull(mechanism, "mechanism");
        mechanism.sysId()
                .rampRateVoltsPerSecond(rampRateVoltsPerSecond)
                .stepVoltage(stepVoltage)
                .timeoutSeconds(timeoutSeconds)
                .voltageLimit(voltageLimit);
    }

    public static void configureNetworkTables(
            Mechanism mechanism,
            String ownerPath,
            double periodSeconds,
            String publishHint) {
        Objects.requireNonNull(mechanism, "mechanism");
        mechanism.networkTables()
                .ownerPath(ownerPath)
                .periodSeconds(periodSeconds)
                .publishHint(publishHint);
    }

    public static void disableForTestMode(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        mechanism.disableAllHooksAndControlLoops();
    }

    public static RegisterableMechanismFactory lazyFactory(
            MechanismConfig<Mechanism> config,
            AtomicInteger buildCount) {
        Objects.requireNonNull(config, "config");
        Objects.requireNonNull(buildCount, "buildCount");
        return new RegisterableMechanismFactory() {
            private RegisterableMechanism cached;

            @Override
            public RegisterableMechanism build() {
                if (cached == null) {
                    cached = config.build();
                    buildCount.incrementAndGet();
                }
                return cached;
            }

            @Override
            public List<Mechanism> flattenForRegistration() {
                RegisterableMechanism entry = build();
                return entry != null ? entry.flattenForRegistration() : List.of();
            }
        };
    }

    public static MechanismVisualizationConfig createVectorLineVisualization(String rootName) {
        String value = Objects.requireNonNull(rootName, "rootName").trim();
        if (value.isEmpty()) {
            throw new IllegalArgumentException("rootName must not be blank");
        }
        return MechanismVisualizationConfig.builder(value)
                .withStaticRootPose(new Pose3d())
                .useVectorLine(true)
                .build();
    }

    public static MechanismPidAutotuners.RelayPositionConfig sanitizedRelayConfig(
            OutputType outputType,
            double relayOutput,
            double errorBand,
            double timeoutSeconds,
            double targetCycles) {
        return new MechanismPidAutotuners.RelayPositionConfig(
                relayOutput,
                errorBand,
                timeoutSeconds,
                targetCycles).sanitized(outputType);
    }

    public static double toVoltage(double percentOutput, double batteryVoltage) {
        return OutputConversions.toMechanismOutput(
                OutputType.VOLTAGE,
                OutputType.PERCENT,
                percentOutput,
                batteryVoltage);
    }

    public static double toPercent(double voltageOutput, double batteryVoltage) {
        return OutputConversions.toMechanismOutput(
                OutputType.PERCENT,
                OutputType.VOLTAGE,
                voltageOutput,
                batteryVoltage);
    }
}
