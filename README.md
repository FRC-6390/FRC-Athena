# VENDOR DEP
`https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/FRC6390-Athena.json`

# Publishing
```bash
export $(cat .env | xargs)
 ```

```bash
 ./gradlew publish -PpublishMode=local # version/year auto-resolve
 ```

 ```bash
export $(cat .env | xargs)
 ./gradlew publish -PpublishMode=online # version/year auto-resolve
 ```

To publish with a specific version, append `-Pversion=<major.minor.patch>` to override the automatic increment. To pin the FRC season manually, append `-PfrcYear=<season>`.

# Simulation Helpers
Mechanism simulations now derive motor types, gearing, and sensor conversions directly from the `MechanismConfig`. Configure physical details once on the config and reuse them everywhere:

```java
MechanismConfig<StatefulElevatorMechanism<ElevatorState>> elevator =
    MechanismConfig
        .statefulElevator(feedforward, ElevatorState.HomeReset)
        .setEncoderConversion(Units.inchesToMeters(1))
        .setEncoderGearRatio(1.0 / 12.0)
        .setSimulationElevator(
            new MechanismConfig.ElevatorSimulationParameters()
                .setCarriageMassKg(25.0)
                .setRangeMeters(0.0, Units.inchesToMeters(70))
                .setDrumRadiusMeters(Units.inchesToMeters(1.5)));
```

Visualization is optional but encouraged. Build 2D/3D hierarchies with `MechanismVisualizationConfig` and attach nodes to the robot base or other moving links:

```java
MechanismVisualizationConfig viz = MechanismVisualizationConfig.builder("ElevatorBase")
    .withStaticRootPose(new Pose3d(0.3, 0.0, 0.2, new Rotation3d()))
    .addNode("Carriage", "ElevatorBase", mech ->
        new Pose3d(0.0, 0.0, Units.inchesToMeters(mech.getPosition()), new Rotation3d()))
    .addNode("Wrist", "Carriage", mech ->
        new Pose3d(0.0, 0.0, 0.15, new Rotation3d(0.0, 0.0, mech.getRotation2d().getRadians())))
    .build();

elevator.setVisualizationConfig(viz);
```

Each `Mechanism` exposes `getMechanism2d()` for Shuffleboard/AdvantageScope and `getMechanism3dPoses()` for logging Pose3d data.

## Sensor Simulation
Digital sensors now participate in mechanism simulation. If a limit switch is given a position via `GenericLimitSwitchConfig#setPosition`, the simulator automatically toggles it based on the mechanism encoder during sim runs. Each sensor's Shuffleboard layout now exposes simulation widgets alongside the existing telemetry:

- `Sim Override` (toggle) – enable manual control
- `Sim Manual Value` (toggle) – desired state when override is true
- `Sim Auto Value` (indicator) – state calculated from the simulation condition
- `Sim Actual Value` (indicator) – value currently driven into the sensor
- `Sim Location` (text) – configured activation position
- `Sim Bound` (debug) – shows whether the hardware supports direct simulation

These widgets appear automatically when the mechanism tab is opened in Shuffleboard, so simulation tweaks live directly next to the real sensor readouts.

For custom sensors, build a `MechanismSensorSimulationConfig` and attach it to the mechanism config:

```java
MechanismSensorSimulationConfig sensors = MechanismSensorSimulationConfig.builder()
    .addDigitalSensor(
        "BottomLimit",
        mech -> mech.getLimitSwitches()[0],
        MechanismSensorSimulationConfig.positionAtOrBelow(() -> Units.inchesToMeters(2.0)),
        () -> Units.inchesToMeters(2.0))
    .addDigitalSensor(
        "BeamBreak",
        mech -> intakeBeamBreak,
        ctx -> ctx.velocity() < 0.01 && intakeHasNoteSupplier.getAsBoolean())
    .build();

mechanismConfig
    .setSensorSimulation(sensors)
    .addLowerLimitSwitch(0, Units.inchesToMeters(2.0), true);
```

Any sensor registered through the config obtains the same simulation widgets described above, letting you script match events or flip sensors by hand without leaving their normal Shuffleboard entry.
