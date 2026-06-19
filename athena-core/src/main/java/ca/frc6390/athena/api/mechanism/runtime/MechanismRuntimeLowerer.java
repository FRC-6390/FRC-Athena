package ca.frc6390.athena.api.mechanism.runtime;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.behavior.automation.MechanismStateHookCallback;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismFeedforward;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopCallback;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopContext;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationPhase;
import ca.frc6390.athena.api.mechanism.definition.MechanismBangBangControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismBooleanInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismCustomControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDigitalInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDoubleInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismEncoderDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardModel;
import ca.frc6390.athena.api.mechanism.definition.MechanismInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismPidControllerDefinition;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfigRecord;
import ca.frc6390.athena.mechanisms.MechanismControlContext;
import ca.frc6390.athena.mechanisms.MechanismContext;
import ca.frc6390.athena.mechanisms.MechanismEncoderSource;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;
import ca.frc6390.athena.mechanisms.MechanismRuntimeConfig;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StatefulMechanismRuntimeConfig;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public final class MechanismRuntimeLowerer {
    private static final double DEFAULT_CONTROL_LOOP_PERIOD_SECONDS = 0.02;
    private static final double TWO_PI = Math.PI * 2.0;

    private MechanismRuntimeLowerer() {
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    public static Mechanism build(MechanismDefinition definition) {
        if (definition.initialState().isPresent()
                || definition.stateType().isPresent()
                || definition.initialStateName().isPresent()) {
            if (definition.stateType().isEmpty() || definition.initialStateName().isEmpty()) {
                throw new IllegalArgumentException("stateful mechanism build requires both stateType and initialStateName");
            }
            Object initialState = definition.initialState()
                .orElseThrow(() -> new IllegalArgumentException(
                    "stateful mechanism build requires an initial state object"));
            MechanismRuntimeConfig runtimeConfig = lower(definition);
            StatefulMechanismRuntimeConfig stateRuntimeConfig = lowerStateful(
                definition,
                initialState,
                new LinkedHashSet<>(runtimeConfig.initiallyDisabledControlLoops()));
            StatefulMechanism mechanism = StatefulMechanism.create(runtimeConfig, initialState, stateRuntimeConfig);
            if (!definition.name().isBlank()) {
                mechanism.setName(definition.name());
            }
            return mechanism;
        }

        MechanismRuntimeConfig<Mechanism> runtimeConfig = lower(definition);
        Mechanism mechanism = Mechanism.create(runtimeConfig);
        if (!definition.name().isBlank()) {
            mechanism.setName(definition.name());
        }
        return mechanism;
    }

    public static MechanismRuntimeConfig<Mechanism> lower(MechanismDefinition definition) {
        List<MotorControllerConfig> motorConfigs = lowerMotors(definition);
        MotorControllerGroup motors = MotorControllerGroup.fromConfigs(motorConfigs.toArray(MotorControllerConfig[]::new));
        Map<String, MechanismEncoderSource> encoderSources = lowerEncoders(definition, motors);
        String defaultSource = defaultEncoderSourceName(definition, encoderSources);
        MechanismConfigRecord data = lowerData(definition, motorConfigs, defaultSource);

        Map<String, Boolean> mutableBoolDefaults = new LinkedHashMap<>();
        Map<String, Double> mutableDoubleDefaults = new LinkedHashMap<>();
        List<GenericLimitSwitchConfig> digitalInputs = new ArrayList<>();
        lowerInputs(definition.inputs(), mutableBoolDefaults, mutableDoubleDefaults, digitalInputs);
        data = data.toBuilder().limitSwitches(digitalInputs).build();

        Map<String, MechanismRuntimeConfig.PidProfile> pidProfiles = new LinkedHashMap<>();
        Map<String, MechanismRuntimeConfig.BangBangProfile> bangBangProfiles = new LinkedHashMap<>();
        Map<String, MechanismRuntimeConfig.FeedforwardProfile> feedforwardProfiles = new LinkedHashMap<>();
        List<MechanismRuntimeConfig.ControlLoopBinding<Mechanism>> controlLoops = new ArrayList<>();
        Set<String> initiallyDisabledControlLoops = new LinkedHashSet<>();
        lowerLoops(
                definition,
                pidProfiles,
                bangBangProfiles,
                feedforwardProfiles,
                controlLoops,
                initiallyDisabledControlLoops);

        return new MechanismRuntimeConfig<>(
                definition.name(),
                definition,
                definition.disabled(),
                motorConfigs,
                encoderSources,
                defaultSource,
                defaultSource,
                defaultSource,
                data,
                List.of(),
                List.of(),
                Map.<String, BooleanSupplier>of(),
                Map.<String, DoubleSupplier>of(),
                Map.<String, IntSupplier>of(),
                Map.<String, Supplier<String>>of(),
                Map.of(),
                Map.of(),
                Map.of(),
                mutableBoolDefaults,
                mutableDoubleDefaults,
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                pidProfiles,
                bangBangProfiles,
                feedforwardProfiles,
                controlLoops,
                initiallyDisabledControlLoops,
                null,
                null,
                null);
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    public static <E> StatefulMechanismRuntimeConfig<StatefulMechanism<E>, E> lowerStateful(
            MechanismDefinition definition,
            E initialState,
            Set<String> initiallyDisabledLoops) {
        Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> enterHooks =
                new LinkedHashMap<>();
        Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> periodicHooks =
                new LinkedHashMap<>();
        Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> exitHooks =
                new LinkedHashMap<>();

        lowerLoopActivationHooks(definition, initialState, initiallyDisabledLoops, enterHooks, exitHooks);
        lowerAutomation(definition, initialState, enterHooks, periodicHooks, exitHooks);

        return new StatefulMechanismRuntimeConfig<>(
                Math.max(0.0, definition.stateMachineDelaySeconds()),
                Map.of(),
                enterHooks,
                periodicHooks,
                exitHooks,
                List.of(),
                List.of(),
                List.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                Map.of(),
                List.of());
    }

    private static List<MotorControllerConfig> lowerMotors(MechanismDefinition definition) {
        List<MotorControllerConfig> configs = new ArrayList<>();
        for (var motor : definition.motors()) {
            int id = requireId(motor.id(), "motor", motor.name());
            MotorControllerConfig config = MotorControllerConfig.create(motor.type().resolveController(), id);
            config.hardware()
                    .canbus(motor.bus().orElse("rio"))
                    .neutralMode(motor.neutralMode().orElse(MotorNeutralMode.Brake))
                    .currentLimit(motor.currentLimit().orElse(40.0));
            configs.add(config);
        }
        return configs;
    }

    private static Map<String, MechanismEncoderSource> lowerEncoders(
            MechanismDefinition definition,
            MotorControllerGroup motors) {
        Map<String, MechanismEncoderSource> sources = new LinkedHashMap<>();
        for (MechanismEncoderDefinition encoder : definition.encoders()) {
            String name = encoder.name();
            if (sources.containsKey(name)) {
                throw new IllegalArgumentException("duplicate encoder name: " + name);
            }
            sources.put(name, lowerEncoder(encoder, motors));
        }
        return Map.copyOf(sources);
    }

    private static MechanismEncoderSource lowerEncoder(
            MechanismEncoderDefinition definition,
            MotorControllerGroup motors) {
        String name = definition.name();
        AthenaEncoder type = definition.type();
        int id = requireId(definition.id(), "encoder", name);
        Encoder device;
        if (type == AthenaEncoder.INTERNAL) {
            device = resolveInternalEncoder(motors, id, definition.bus().orElse("rio"), name);
        } else {
            EncoderConfig config = EncoderConfig.create(type.resolve(), id);
            config.hardware().canbus(definition.bus().orElse("rio"));
            definition.gearRatio().ifPresent(value -> config.measurement().gearRatio(value));
            definition.conversion().ifPresent(value -> config.measurement().conversion(value));
            definition.offset().ifPresent(value -> config.measurement().offset(value));
            definition.conversionOffset().ifPresent(value -> config.measurement().conversionOffset(value));
            device = HardwareFactories.encoder(config);
        }
        definition.gearRatio().ifPresent(device::setGearRatio);
        definition.conversion().ifPresent(device::setConversion);
        definition.offset().ifPresent(device::setOffset);
        definition.conversionOffset().ifPresent(device::setConversionOffset);
        return new MechanismEncoderSource(
                name,
                device,
                definition.unit().orElse(MechanismEncoderUnit.ROTATIONS),
                definition.wrapsEvery().orElse(Double.NaN),
                true,
                true,
                true);
    }

    private static Encoder resolveInternalEncoder(
            MotorControllerGroup motors,
            int id,
            String bus,
            String encoderName) {
        String resolvedBus = bus == null || bus.isBlank() ? "rio" : bus;
        for (MotorController controller : motors.getControllers()) {
            if (controller == null) {
                continue;
            }
            if (Math.abs(controller.getId()) != Math.abs(id)) {
                continue;
            }
            String controllerBus = controller.getCanbus();
            if (controllerBus == null || controllerBus.isBlank()) {
                controllerBus = "rio";
            }
            if (!resolvedBus.equals(controllerBus)) {
                continue;
            }
            Encoder encoder = controller.getEncoder();
            if (encoder != null) {
                return encoder;
            }
        }
        throw new IllegalStateException(
                "Encoder source '" + encoderName + "' uses INTERNAL but no matching motor encoder exists for id "
                        + Math.abs(id)
                        + " on canbus '"
                        + resolvedBus
                        + "'");
    }

    private static String defaultEncoderSourceName(
            MechanismDefinition definition,
            Map<String, MechanismEncoderSource> encoderSources) {
        if (encoderSources.isEmpty()) {
            return null;
        }
        for (MechanismEncoderDefinition encoder : definition.encoders()) {
            if (encoder.defaultPositionSource()) {
                return encoder.name();
            }
        }
        return encoderSources.keySet().iterator().next();
    }

    private static MechanismConfigRecord lowerData(
            MechanismDefinition definition,
            List<MotorControllerConfig> motors,
            String defaultSource) {
        OutputType inferredOutput = inferOutputType(definition.loops());
        MechanismConfigRecord.Builder builder = MechanismConfigRecord.defaults()
                .toBuilder()
                .motors(new ArrayList<>(motors))
                .positionSource(defaultSource)
                .velocitySource(defaultSource)
                .absoluteSource(defaultSource)
                .outputType(inferredOutput)
                .useVoltage(inferredOutput == OutputType.VOLTAGE);

        definition.identity().travelRange().ifPresent(range -> builder.minBound(range.min()).maxBound(range.max()));
        MechanismEncoderDefinition defaultEncoder = definition.encoders().stream()
                .filter(MechanismEncoderDefinition::defaultPositionSource)
                .findFirst()
                .orElseGet(() -> definition.encoders().isEmpty() ? null : definition.encoders().getFirst());
        if (defaultEncoder != null) {
            defaultEncoder.gearRatio().ifPresent(builder::encoderGearRatio);
            defaultEncoder.conversion().ifPresent(builder::encoderConversion);
            defaultEncoder.offset().ifPresent(builder::encoderOffset);
            defaultEncoder.conversionOffset().ifPresent(builder::encoderConversionOffset);
        }
        if (definition.identity().continuousRotation()) {
            continuousInputRange(definition).ifPresent(range -> builder
                    .pidContinous(true)
                    .continousMin(range[0])
                    .continousMax(range[1]));
        }
        return builder.build();
    }

    private static OutputType inferOutputType(List<MechanismLoopDefinition> loops) {
        boolean sawPercent = false;
        boolean sawVoltage = false;
        for (MechanismLoopDefinition loop : loops) {
            if (loop.output() == OutputType.VOLTAGE) {
                sawVoltage = true;
            } else {
                sawPercent = true;
            }
        }
        return sawVoltage && !sawPercent ? OutputType.VOLTAGE : OutputType.PERCENT;
    }

    private static Optional<double[]> continuousInputRange(MechanismDefinition definition) {
        return definition.identity().positionDomain().flatMap(domain -> {
            PositionUnit unit = domain.units();
            return switch (unit) {
                case DEGREES -> Optional.of(new double[] {-180.0, 180.0});
                case RADIANS -> Optional.of(new double[] {-Math.PI, Math.PI});
                case ROTATIONS -> Optional.of(new double[] {-0.5, 0.5});
                default -> Optional.empty();
            };
        });
    }

    private static void lowerInputs(
            List<MechanismInputDefinition> definitions,
            Map<String, Boolean> mutableBoolDefaults,
            Map<String, Double> mutableDoubleDefaults,
            List<GenericLimitSwitchConfig> digitalInputs) {
        for (MechanismInputDefinition input : definitions) {
            if (input instanceof MechanismBooleanInputDefinition boolInput) {
                boolInput.defaultValue().ifPresent(value -> mutableBoolDefaults.put(boolInput.name(), value));
            } else if (input instanceof MechanismDoubleInputDefinition doubleInput) {
                if (doubleInput.defaultValue().isPresent()) {
                    mutableDoubleDefaults.put(doubleInput.name(), doubleInput.defaultValue().getAsDouble());
                }
            } else if (input instanceof MechanismDigitalInputDefinition digitalInput) {
                int signedPort = digitalInput.inverted() ? -digitalInput.port() : digitalInput.port();
                GenericLimitSwitchConfig config = GenericLimitSwitchConfig.create(signedPort).name(digitalInput.name());
                if (digitalInput.position().isPresent()) {
                    config = config.position(digitalInput.position().getAsDouble());
                }
                if (digitalInput.hardstop()) {
                    config = config.hardstop(true, digitalInput.blockDirection());
                }
                if (Double.isFinite(digitalInput.delaySeconds()) && digitalInput.delaySeconds() > 0.0) {
                    config = config.delay(digitalInput.delaySeconds());
                }
                digitalInputs.add(config);
            }
        }
    }

    private static void lowerLoops(
            MechanismDefinition definition,
            Map<String, MechanismRuntimeConfig.PidProfile> pidProfiles,
            Map<String, MechanismRuntimeConfig.BangBangProfile> bangBangProfiles,
            Map<String, MechanismRuntimeConfig.FeedforwardProfile> feedforwardProfiles,
            List<MechanismRuntimeConfig.ControlLoopBinding<Mechanism>> controlLoops,
            Set<String> initiallyDisabledControlLoops) {
        for (MechanismLoopDefinition loop : definition.loops()) {
            String name = loop.name();
            if (loop.controller() instanceof MechanismPidControllerDefinition pid) {
                registerUnique(pidProfiles, name, new MechanismRuntimeConfig.PidProfile(
                        loop.output(),
                        pid.kP(),
                        pid.kI(),
                        pid.kD(),
                        0.0,
                        pid.tolerance().orElse(0.0),
                        pid.maxVelocity().orElse(Double.NaN),
                        pid.maxAcceleration().orElse(Double.NaN),
                        MechanismRuntimeConfig.PidAutotunerConfig.defaults(),
                        pid.inputSource(),
                        pid.setpointSource()));
                controlLoops.add(new MechanismRuntimeConfig.ControlLoopBinding<>(
                        name,
                        DEFAULT_CONTROL_LOOP_PERIOD_SECONDS,
                        ctx -> ctx.pidOut(
                                name,
                                resolveInputSource(ctx, pid.inputSource()),
                                resolveSetpointSource(ctx, pid.setpointSource()))));
            } else if (loop.controller() instanceof MechanismBangBangControllerDefinition bangBang) {
                registerUnique(bangBangProfiles, name, new MechanismRuntimeConfig.BangBangProfile(
                        loop.output(),
                        bangBang.highOutput(),
                        bangBang.lowOutput(),
                        bangBang.tolerance().orElse(0.0),
                        bangBang.inputSource(),
                        bangBang.setpointSource()));
                controlLoops.add(new MechanismRuntimeConfig.ControlLoopBinding<>(
                        name,
                        DEFAULT_CONTROL_LOOP_PERIOD_SECONDS,
                        ctx -> ctx.bangBangOut(
                                name,
                                resolveInputSource(ctx, bangBang.inputSource()),
                                resolveSetpointSource(ctx, bangBang.setpointSource()))));
            } else if (loop.controller() instanceof MechanismFeedforwardControllerDefinition feedforward) {
                boolean useMechanismSetpointForFeedforward = definition.identity().positionDomain()
                        .map(domain -> domain.kind() == PositionDomainKind.VELOCITY)
                        .orElse(false);
                registerUnique(feedforwardProfiles, name, new MechanismRuntimeConfig.FeedforwardProfile(
                        loop.output(),
                        switch (feedforward.model()) {
                            case SIMPLE -> MechanismRuntimeConfig.FeedforwardType.SIMPLE;
                            case ARM -> MechanismRuntimeConfig.FeedforwardType.ARM;
                            case ELEVATOR -> MechanismRuntimeConfig.FeedforwardType.ELEVATOR;
                        },
                        feedforward.kS(),
                        feedforward.kG(),
                        feedforward.kV(),
                        feedforward.kA(),
                        feedforward.tolerance().orElse(0.0),
                        feedforward.setpointSource()));
                controlLoops.add(new MechanismRuntimeConfig.ControlLoopBinding<>(
                        name,
                        DEFAULT_CONTROL_LOOP_PERIOD_SECONDS,
                        ctx -> ctx.feedforwardOut(
                                name,
                                resolveInputSource(ctx, null),
                                resolveSetpointSource(ctx, feedforward.setpointSource()),
                                resolveFeedforwardSetpoint(
                                        ctx,
                                        feedforward.setpointSource(),
                                        useMechanismSetpointForFeedforward))));
            } else if (loop.controller() instanceof MechanismCustomControllerDefinition custom) {
                MechanismLoopCallback callback = custom.callback().orElseThrow(() ->
                        new IllegalArgumentException("custom control loop '" + name + "' is missing a callback"));
                controlLoops.add(new MechanismRuntimeConfig.ControlLoopBinding<>(
                        name,
                        DEFAULT_CONTROL_LOOP_PERIOD_SECONDS,
                        ctx -> callback.calculate(new RuntimeLoopContextAdapter(ctx))));
            } else {
                throw new IllegalArgumentException("unsupported loop controller: " + loop.controller().getClass().getName());
            }

            if (loop.activation().mode() == LoopMode.MANUAL) {
                initiallyDisabledControlLoops.add(name);
                continue;
            }
            if (loop.activation().mode() != LoopMode.ENABLED) {
                throw new IllegalArgumentException("unsupported loop mode: " + loop.activation().mode());
            }
        }
    }

    private static double resolveInputSource(
            MechanismControlContext<?> context,
            ca.frc6390.athena.mechanisms.MechanismInputSource source) {
        ca.frc6390.athena.mechanisms.MechanismInputSource resolved =
                source != null ? source : ca.frc6390.athena.mechanisms.MechanismInputSource.Position;
        return switch (resolved.kind()) {
            case POSITION -> resolved.encoderId() != null
                    ? context.mechanism().position(resolved.encoderId())
                    : context.mechanism().position();
            case VELOCITY -> resolved.encoderId() != null
                    ? context.mechanism().velocity(resolved.encoderId())
                    : context.mechanism().velocity();
            case ABSOLUTE -> resolved.encoderId() != null
                    ? context.mechanism().absolutePosition(resolved.encoderId())
                    : context.mechanism().absolutePosition();
            case INPUT -> context.doubleInput(resolved.inputKey());
        };
    }

    private static double resolveSetpointSource(
            MechanismControlContext<?> context,
            ca.frc6390.athena.mechanisms.MechanismSetpointSource source) {
        ca.frc6390.athena.mechanisms.MechanismSetpointSource resolved =
                source != null ? source : ca.frc6390.athena.mechanisms.MechanismSetpointSource.Setpoint;
        return switch (resolved.kind()) {
            case SETPOINT -> context.mechanism().setpoint() + context.mechanism().nudge();
            case INPUT -> context.doubleInput(resolved.inputKey());
        };
    }

    private static double resolveFeedforwardSetpoint(
            MechanismControlContext<?> context,
            ca.frc6390.athena.mechanisms.MechanismSetpointSource source,
            boolean useMechanismSetpoint) {
        if (source != null && source.kind() == ca.frc6390.athena.mechanisms.MechanismSetpointSource.Kind.INPUT) {
            return context.doubleInput(source.inputKey());
        }
        return useMechanismSetpoint ? context.mechanism().setpoint() + context.mechanism().nudge() : 0.0;
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <E> void lowerLoopActivationHooks(
            MechanismDefinition definition,
            E initialState,
            Set<String> initiallyDisabledLoops,
            Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> enterHooks,
            Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> exitHooks) {
        for (MechanismLoopDefinition loop : definition.loops()) {
            if (loop.activation().mode() != LoopMode.ENABLED || loop.activation().states().isEmpty()) {
                continue;
            }
            List<Object> states = resolveStates(initialState, loop.activation().states(), "loop " + loop.name());
            boolean initiallyActive = false;
            for (Object state : states) {
                if (state.equals(initialState)) {
                    initiallyActive = true;
                    break;
                }
            }
            if (!initiallyActive) {
                initiallyDisabledLoops.add(loop.name());
            }
            StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E> enable =
                    (MechanismContext<StatefulMechanism<E>, E> ctx) -> ctx.enableControlLoop(loop.name());
            StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E> disable =
                    (MechanismContext<StatefulMechanism<E>, E> ctx) -> ctx.disableControlLoop(loop.name());
            addStateHooks(enterHooks, states, enable);
            addStateHooks(exitHooks, states, disable);
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <E> void lowerAutomation(
            MechanismDefinition definition,
            E initialState,
            Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> enterHooks,
            Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> periodicHooks,
            Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E>>> exitHooks) {
        for (MechanismAutomationDefinition automation : definition.automation()) {
            List<Object> states = resolveStates(initialState, automation.states(), automation.phase().name());
            StatefulMechanismRuntimeConfig.StateHook<StatefulMechanism<E>, E> hook =
                    ctx -> automation.callback().apply(ctx);
            switch (automation.phase()) {
                case STATE_ENTER -> addStateHooks(enterHooks, states, hook);
                case STATE_PERIODIC -> addStateHooks(periodicHooks, states, hook);
                case STATE_EXIT -> addStateHooks(exitHooks, states, hook);
                default -> throw new IllegalArgumentException("unsupported automation phase: " + automation.phase());
            }
        }
    }

    private static <T> void registerUnique(Map<String, T> map, String name, T value) {
        if (map.containsKey(name)) {
            throw new IllegalArgumentException("duplicate controller name: " + name);
        }
        map.put(name, value);
    }

    @SuppressWarnings("unchecked")
    private static <E, T extends Mechanism> void addStateHooks(
            Map<Object, List<StatefulMechanismRuntimeConfig.StateHook<T, E>>> target,
            List<Object> states,
            StatefulMechanismRuntimeConfig.StateHook<T, E> hook) {
        for (Object state : states) {
            target.computeIfAbsent(state, ignored -> new ArrayList<>()).add((StatefulMechanismRuntimeConfig.StateHook<T, E>) hook);
        }
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static <E> List<Object> resolveStates(
            E initialState,
            List<String> states,
            String owner) {
        List<Object> resolved = new ArrayList<>();
        for (String state : states) {
            Object resolvedState = ca.frc6390.athena.mechanisms.statespec.StateSpecAccess.resolve(initialState, state);
            if (resolvedState == null) {
                throw new IllegalArgumentException(owner + " references unknown state '" + state + "'");
            }
            resolved.add(resolvedState);
        }
        if (resolved.isEmpty()) {
            throw new IllegalArgumentException(owner + " must reference at least one state");
        }
        return resolved;
    }

    private static int requireId(OptionalInt id, String kind, String name) {
        if (id.isEmpty()) {
            throw new IllegalArgumentException(kind + " '" + name + "' is missing a hardware id");
        }
        return id.getAsInt();
    }

    private static final class RuntimeLoopContextAdapter implements MechanismLoopContext {
        private final MechanismControlContext<?> legacy;

        private RuntimeLoopContextAdapter(MechanismControlContext<?> legacy) {
            this.legacy = legacy;
        }

        @Override
        public RobotCore<?> robotCore() {
            return legacy.robotCore();
        }

        @Override
        public Mechanism mechanism() {
            return legacy.mechanism();
        }

        @Override
        public double controlLoopDtSeconds() {
            return legacy.controlLoopDtSeconds();
        }

        @Override
        public double setpoint() {
            return legacy.setpoint();
        }

        @Override
        public Object state() {
            return legacy.state();
        }

        @Override
        public boolean input(String key) {
            return legacy.input(key);
        }

        @Override
        public double doubleInput(String key) {
            return legacy.doubleInput(key);
        }

        @Override
        public int intVal(String key) {
            return legacy.intVal(key);
        }

        @Override
        public String stringVal(String key) {
            return legacy.stringVal(key);
        }

        @Override
        public <V> V objectInput(String key, Class<V> type) {
            return legacy.objectInput(key, type);
        }

        @Override
        public double calculate(MechanismPid pid, double measurement, double setpoint) {
            return legacy.pidOut(pid.definition().name(), measurement, setpoint);
        }

        @Override
        public double calculate(MechanismFeedforward feedforward, double velocity) {
            return legacy.feedforwardOut(feedforward.definition().name(), velocity);
        }

        @Override
        public double calculate(MechanismFeedforward feedforward, double measurement, double setpoint, double velocity) {
            return legacy.feedforwardOut(feedforward.definition().name(), measurement, setpoint, velocity);
        }

        @Override
        public double calculate(MechanismFeedforward feedforward, double currentVelocity, double nextVelocity) {
            return legacy.feedforwardOut(feedforward.definition().name(), currentVelocity, nextVelocity);
        }

        @Override
        public double calculate(
                MechanismFeedforward feedforward,
                double measurement,
                double setpoint,
                double currentVelocity,
                double nextVelocity) {
            return legacy.feedforwardOut(feedforward.definition().name(), measurement, setpoint, currentVelocity, nextVelocity);
        }
    }
}
