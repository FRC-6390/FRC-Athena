package ca.frc6390.athena.mechanisms.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.OptionalDouble;

import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.sensors.EnhancedDigitalInput;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Runtime manager that applies {@link MechanismSensorSimulationConfig} to a {@link Mechanism}. When
 * running in simulation it drives the underlying digital inputs, exposes manual overrides via
 * NetworkTables (no Shuffleboard containers), and allows dashboards to toggle sensor states
 * without redeploying.
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
        GenericLimitSwitch[] switches = mechanism.limitSwitches();
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
                        mechanism.position(),
                        mechanism.velocity());
        for (SensorRuntime sensor : sensors) {
            sensor.update(context);
        }
    }

    private static final class SensorRuntime {

        private final Mechanism mechanism;
        private RobotNetworkTables.Node baseNode;
        private final String name;
        private final EnhancedDigitalInput sensor;
        private final MechanismSensorSimulationConfig.SensorCondition condition;
        private final boolean supportsSimulation;
        private final OptionalDouble location;
        private NetworkTableEntry overrideEntry;
        private NetworkTableEntry manualEntry;
        private NetworkTableEntry autoEntry;
        private NetworkTableEntry actualEntry;
        private NetworkTableEntry locationEntry;
        private NetworkTableEntry supportsSimulationEntry;
        private boolean overrideValue;
        private boolean manualValue;
        private boolean autoValue;
        private boolean actualValue;

        private SensorRuntime(Mechanism mechanism,
                              String name,
                              EnhancedDigitalInput sensor,
                              MechanismSensorSimulationConfig.SensorCondition condition,
                              OptionalDouble location) {
            this.mechanism = mechanism;
            this.name = Objects.requireNonNullElse(name, sensor.getClass().getSimpleName());
            this.sensor = sensor;
            this.condition = condition;
            this.supportsSimulation = sensor.supportsSimulation();
            this.location = location;
            this.baseNode = null;
            this.overrideEntry = null;
            this.manualEntry = null;
            this.autoEntry = null;
            this.actualEntry = null;
            this.locationEntry = null;
            this.supportsSimulationEntry = null;
            this.overrideValue = false;
            this.manualValue = false;
            this.autoValue = false;
            this.actualValue = false;
        }

        void update(MechanismSensorSimulationConfig.EvaluationContext context) {
            ensureEntries();
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
        private void ensureEntries() {
            if (overrideEntry != null) {
                return;
            }
            if (baseNode == null) {
                baseNode = resolveBaseNode(mechanism, name);
            }
            if (baseNode == null) {
                return;
            }
            // These are plain NetworkTables entries so dashboards can toggle them directly.
            overrideEntry = initIfAbsent(baseNode.entry("override"), false);
            manualEntry = initIfAbsent(baseNode.entry("manual"), false);
            autoEntry = initIfAbsent(baseNode.entry("auto"), false);
            actualEntry = initIfAbsent(baseNode.entry("actual"), false);
            locationEntry = initIfAbsent(baseNode.entry("location"), location.orElse(Double.NaN));
            supportsSimulationEntry = initIfAbsent(baseNode.entry("supportsSimulation"), supportsSimulation);

            // Push initial values once to ensure the tree exists in dashboards.
            autoEntry.setBoolean(autoValue);
            actualEntry.setBoolean(actualValue);
            locationEntry.setDouble(location.orElse(Double.NaN));
            supportsSimulationEntry.setBoolean(supportsSimulation);
        }

	        private static RobotNetworkTables.Node resolveBaseNode(Mechanism mechanism, String sensorName) {
	            if (mechanism == null || sensorName == null || sensorName.isBlank()) {
	                return null;
	            }

            ca.frc6390.athena.core.RobotCore<?> core = mechanism.getRobotCore();
            RobotNetworkTables nt = core != null ? core.networkTables() : null;
	            if (nt == null) {
	                return null;
	            }

	            // Put simulation config alongside the mechanism itself:
	            // Athena/Mechanisms/.../<MechName>/Simulation/Sensors/<SensorName>/...
	            RobotNetworkTables.Node mechNode =
	                    mechanism.networkTables().resolveNode(nt.root().child("Mechanisms"));
	            return mechNode
	                    .child("Simulation")
	                    .child("Sensors")
	                    .child(sanitizeSegment(sensorName));
	        }

        private static String sanitizeSegment(String raw) {
            String s = raw == null ? "" : raw.trim();
            if (s.isEmpty()) {
                return "";
            }
            s = s.replace('\\', '/');
            s = s.replaceAll("/+", "/");
            s = s.replace('/', '_');
            return s.replace(' ', '_');
        }

        private static NetworkTableEntry initIfAbsent(NetworkTableEntry entry, boolean defaultValue) {
            if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
                entry.setBoolean(defaultValue);
            }
            return entry;
        }

        private static NetworkTableEntry initIfAbsent(NetworkTableEntry entry, double defaultValue) {
            if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
                entry.setDouble(defaultValue);
            }
            return entry;
        }
    }
}
