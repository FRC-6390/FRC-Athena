package ca.frc6390.athena.mechanisms.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;

import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.sensors.EnhancedDigitalInput;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Runtime manager that applies {@link MechanismSensorSimulationConfig} to a {@link Mechanism}. When
 * running in simulation it drives the underlying digital inputs, exposes manual overrides via
 * NetworkTables, and adds quick commands for toggling sensor states.
 */
public final class MechanismSensorSimulation {

    private static final MechanismSensorSimulation EMPTY =
            new MechanismSensorSimulation(List.of());

    private final List<SensorRuntime> sensors;

    private MechanismSensorSimulation(List<SensorRuntime> sensors) {
        this.sensors = sensors;
    }

    public static MechanismSensorSimulation fromConfig(Mechanism mechanism,
                                                       MechanismSensorSimulationConfig config) {
        if (mechanism == null || config == null || config.sensors().isEmpty()) {
            return EMPTY;
        }
        List<SensorRuntime> runtimes = new ArrayList<>();
        for (MechanismSensorSimulationConfig.SensorDefinition definition : config.sensors()) {
            EnhancedDigitalInput sensor = definition.accessor().apply(mechanism);
            if (sensor == null) {
                continue;
            }
            runtimes.add(new SensorRuntime(
                    mechanism,
                    definition.name(),
                    sensor,
                    definition.condition(),
                    definition.location()));
        }
        return runtimes.isEmpty() ? EMPTY : new MechanismSensorSimulation(List.copyOf(runtimes));
    }

    public static MechanismSensorSimulation forLimitSwitches(Mechanism mechanism) {
        if (mechanism == null) {
            return EMPTY;
        }
        GenericLimitSwitch[] switches = mechanism.getLimitSwitches();
        if (switches == null || switches.length == 0) {
            return EMPTY;
        }

        List<SensorRuntime> runtimes = new ArrayList<>();
        for (GenericLimitSwitch limitSwitch : switches) {
            if (limitSwitch == null) {
                continue;
            }
            double position = limitSwitch.getPosition();
            if (!Double.isFinite(position)) {
                continue;
            }
            int direction = limitSwitch.getBlockDirectionMultiplier();
            MechanismSensorSimulationConfig.SensorCondition condition;
            if (direction > 0) {
                condition = MechanismSensorSimulationConfig.positionAtOrAbove(() -> position);
            } else if (direction < 0) {
                condition = MechanismSensorSimulationConfig.positionAtOrBelow(() -> position);
            } else {
                double tolerance = 1e-3;
                condition = ctx -> Math.abs(ctx.position() - position) <= tolerance;
            }
            String name = "LimitSwitch-" + limitSwitch.getPort();
            runtimes.add(new SensorRuntime(
                    mechanism,
                    name,
                    limitSwitch,
                    condition,
                    OptionalDouble.of(position)));
        }

        return runtimes.isEmpty() ? EMPTY : new MechanismSensorSimulation(List.copyOf(runtimes));
    }

    public static MechanismSensorSimulation empty() {
        return EMPTY;
    }

    public boolean isEmpty() {
        return sensors.isEmpty();
    }

    public void update(Mechanism mechanism) {
        if (!RobotBase.isSimulation() || sensors.isEmpty() || mechanism == null) {
            return;
        }
        MechanismSensorSimulationConfig.EvaluationContext context =
                new MechanismSensorSimulationConfig.EvaluationContext(
                        mechanism,
                        mechanism.getPosition(),
                        mechanism.getVelocity());
        for (SensorRuntime sensor : sensors) {
            sensor.update(context);
        }
    }

    public void decorateSensorLayout(EnhancedDigitalInput sensor,
                                     ShuffleboardLayout layout,
                                     SendableLevel level) {
        if (!RobotBase.isSimulation() || sensors.isEmpty() || sensor == null || layout == null) {
            return;
        }
        sensors.stream()
                .filter(runtime -> runtime.isSensor(sensor))
                .findFirst()
                .ifPresent(runtime -> runtime.decorateLayout(layout, level));
    }

    private static final class SensorRuntime {

        private final String name;
        private final EnhancedDigitalInput sensor;
        private final MechanismSensorSimulationConfig.SensorCondition condition;
        private final boolean supportsSimulation;
        private final OptionalDouble location;
        private GenericEntry overrideEntry;
        private GenericEntry manualEntry;
        private GenericEntry autoEntry;
        private GenericEntry actualEntry;
        private GenericEntry locationEntry;
        private GenericEntry simBoundEntry;
        private boolean overrideValue;
        private boolean manualValue;
        private boolean autoValue;
        private boolean actualValue;

        private SensorRuntime(Mechanism mechanism,
                              String name,
                              EnhancedDigitalInput sensor,
                              MechanismSensorSimulationConfig.SensorCondition condition,
                              OptionalDouble location) {
            this.name = Objects.requireNonNullElse(name, sensor.getClass().getSimpleName());
            this.sensor = sensor;
            this.condition = condition;
            this.supportsSimulation = sensor.supportsSimulation();
            this.location = location;
            this.overrideEntry = null;
            this.manualEntry = null;
            this.autoEntry = null;
            this.actualEntry = null;
            this.locationEntry = null;
            this.simBoundEntry = null;
            this.overrideValue = false;
            this.manualValue = false;
            this.autoValue = false;
            this.actualValue = false;
        }

        void update(MechanismSensorSimulationConfig.EvaluationContext context) {
            boolean computedAuto = condition.test(context);
            setAutoValue(computedAuto);

            overrideValue = readOverride();
            manualValue = readManual();

            boolean finalValue = overrideValue ? manualValue : autoValue;
            updateActualEntry(finalValue);

            if (supportsSimulation) {
                sensor.setSimulatedTriggered(finalValue);
            }
        }

        boolean isSensor(EnhancedDigitalInput candidate) {
            return sensor == candidate;
        }

        void decorateLayout(ShuffleboardLayout layout, SendableLevel level) {
            if (layout == null) {
                return;
            }
            overrideEntry = ensureToggle(layout, "Sim Override", overrideEntry, overrideValue);
            manualEntry = ensureToggle(layout, "Sim Manual Value", manualEntry, manualValue);
            autoEntry = ensureIndicator(layout, "Sim Auto Value", autoEntry, autoValue);
            actualEntry = ensureIndicator(layout, "Sim Actual Value", actualEntry, actualValue);
            locationEntry = ensureLocation(layout, locationEntry);
            if (level == SendableLevel.DEBUG) {
                simBoundEntry = ensureIndicator(layout, "Sim Bound", simBoundEntry, supportsSimulation);
            }
            pushEntryValues();
        }

        private boolean readOverride() {
            return overrideEntry != null ? overrideEntry.getBoolean(overrideValue) : overrideValue;
        }

        private boolean readManual() {
            return manualEntry != null ? manualEntry.getBoolean(manualValue) : manualValue;
        }

        private void setAutoValue(boolean value) {
            autoValue = value;
            if (autoEntry != null) {
                autoEntry.setBoolean(value);
            }
            updateActualEntry(overrideValue ? manualValue : autoValue);
        }

        private void updateActualEntry(boolean value) {
            actualValue = value;
            if (actualEntry != null) {
                actualEntry.setBoolean(value);
            }
        }

        private GenericEntry ensureToggle(ShuffleboardLayout layout,
                                          String title,
                                          GenericEntry entry,
                                          boolean defaultValue) {
            if (entry == null || !entry.exists()) {
                entry = layout
                        .add(title, defaultValue)
                        .withWidget(BuiltInWidgets.kToggleSwitch)
                        .getEntry();
            }
            return entry;
        }

        private GenericEntry ensureIndicator(ShuffleboardLayout layout,
                                             String title,
                                             GenericEntry entry,
                                             boolean defaultValue) {
            if (entry == null || !entry.exists()) {
                entry = layout
                        .add(title, defaultValue)
                        .withWidget(BuiltInWidgets.kBooleanBox)
                        .getEntry();
            }
            return entry;
        }

        private GenericEntry ensureLocation(ShuffleboardLayout layout, GenericEntry entry) {
            if (entry == null || !entry.exists()) {
                entry = layout
                        .add("Sim Location", location.orElse(Double.NaN))
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
            }
            return entry;
        }

        private void pushEntryValues() {
            if (overrideEntry != null) {
                overrideEntry.setBoolean(overrideValue);
            }
            if (manualEntry != null) {
                manualEntry.setBoolean(manualValue);
            }
            if (autoEntry != null) {
                autoEntry.setBoolean(autoValue);
            }
            if (actualEntry != null) {
                actualEntry.setBoolean(actualValue);
            }
            if (locationEntry != null) {
                locationEntry.setDouble(location.orElse(Double.NaN));
            }
            if (simBoundEntry != null) {
                simBoundEntry.setBoolean(supportsSimulation);
            }
        }
    }
}
