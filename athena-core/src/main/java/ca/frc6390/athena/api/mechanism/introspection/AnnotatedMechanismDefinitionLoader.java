package ca.frc6390.athena.api.mechanism.introspection;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateEnter;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateExit;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStatePeriodic;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopSchedule;
import ca.frc6390.athena.api.mechanism.annotation.encoder.DefaultPositionSource;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.identity.ContinuousRotation;
import ca.frc6390.athena.api.mechanism.annotation.identity.InitialState;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.definition.LoopActivation;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationPhase;
import ca.frc6390.athena.api.mechanism.definition.MechanismBooleanInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismCustomControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDigitalInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDoubleInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismEncoderDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismInputDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismMotorDefinition;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismControlLoop;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismFeedforward;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.StatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.identity.MechanismIdentity;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainSpec;
import ca.frc6390.athena.api.mechanism.identity.TravelRange;
import ca.frc6390.athena.api.mechanism.input.MechanismBooleanInput;
import ca.frc6390.athena.api.mechanism.input.MechanismDoubleInput;
import ca.frc6390.athena.api.mechanism.annotation.input.BooleanInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DigitalInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DoubleInput;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class AnnotatedMechanismDefinitionLoader {
    private AnnotatedMechanismDefinitionLoader() {
    }

    public static MechanismDefinition load(Class<?> type) {
        return load(type, AnnotatedMechanismCallbackFactory.declarationInstance(type));
    }

    public static MechanismDefinition load(Object declaration) {
        if (declaration instanceof Class<?> type) {
            return load(type);
        }
        Class<?> type = MechanismDeclarationTypeResolver.resolve(declaration);
        return load(type, Optional.of(declaration));
    }

    private static MechanismDefinition load(Class<?> type, Optional<Object> declaration) {
        String name = mechanismName(type);
        MechanismIdentity identity = identity(type);

        List<MechanismMotorDefinition> motors = new ArrayList<>();
        List<MechanismEncoderDefinition> encoders = new ArrayList<>();
        List<MechanismInputDefinition> inputs = new ArrayList<>();
        List<MechanismLoopDefinition> loops = new ArrayList<>();
        List<MechanismAutomationDefinition> automation = new ArrayList<>();
        InitialState initialState = type.getAnnotation(InitialState.class);

        for (Field field : type.getDeclaredFields()) {
            Motor motor = field.getAnnotation(Motor.class);
            if (motor != null) {
                motors.add(new MechanismMotorDefinition(
                    DeclarationNameResolver.publicName(field, motor),
                    motor.type(),
                    hardwareId(motor.id()),
                    nonBlank(motor.bus()),
                    finite(motor.currentLimit()),
                    Optional.of(motor.neutralMode()),
                    field.getType()));
            }

            Encoder encoder = field.getAnnotation(Encoder.class);
            if (encoder != null) {
                encoders.add(new MechanismEncoderDefinition(
                    DeclarationNameResolver.publicName(field, encoder),
                    encoder.type(),
                    hardwareId(encoder.id()),
                    nonBlank(encoder.bus()),
                    finite(encoder.gearRatio()),
                    finite(encoder.conversion()),
                    finite(encoder.offset()),
                    finite(encoder.conversionOffset()),
                    Optional.of(encoder.unit()),
                    finite(encoder.wrapsEvery()),
                    field.isAnnotationPresent(DefaultPositionSource.class),
                    field.getType()));
            }

            DigitalInput digitalInput = field.getAnnotation(DigitalInput.class);
            if (digitalInput != null) {
                int rawPort = digitalInput.port();
                inputs.add(new MechanismDigitalInputDefinition(
                    DeclarationNameResolver.publicName(field, digitalInput),
                    Math.abs(rawPort),
                    rawPort < 0,
                    finite(digitalInput.position()),
                    digitalInput.hardstop(),
                    digitalInput.blockDirection(),
                    digitalInput.delaySeconds(),
                    field.getType()));
            }

            BooleanInput booleanInput = field.getAnnotation(BooleanInput.class);
            if (booleanInput != null) {
                String publicName = DeclarationNameResolver.publicName(field, booleanInput);
                Optional<Boolean> defaultValue = fieldValue(field, declaration)
                    .filter(MechanismBooleanInput.class::isInstance)
                    .map(MechanismBooleanInput.class::cast)
                    .map(input -> input.named(publicName))
                    .map(MechanismBooleanInput::definition)
                    .flatMap(MechanismBooleanInputDefinition::defaultValue);
                inputs.add(new MechanismBooleanInputDefinition(
                    publicName,
                    defaultValue,
                    field.getType()));
            }

            DoubleInput doubleInput = field.getAnnotation(DoubleInput.class);
            if (doubleInput != null) {
                String publicName = DeclarationNameResolver.publicName(field, doubleInput);
                java.util.OptionalDouble defaultValue = fieldValue(field, declaration)
                    .filter(MechanismDoubleInput.class::isInstance)
                    .map(MechanismDoubleInput.class::cast)
                    .map(input -> input.named(publicName))
                    .map(MechanismDoubleInput::definition)
                    .map(MechanismDoubleInputDefinition::defaultValue)
                    .orElseGet(java.util.OptionalDouble::empty);
                inputs.add(new MechanismDoubleInputDefinition(
                    publicName,
                    defaultValue,
                    field.getType()));
            }

            ControlLoop controlLoop = field.getAnnotation(ControlLoop.class);
            if (controlLoop != null) {
                String publicName = DeclarationNameResolver.publicName(field, controlLoop);
                MechanismLoopDefinition declared = fieldLoopDefinition(field, declaration, publicName)
                    .map(definition -> new MechanismLoopDefinition(
                        publicName,
                        controlLoop.output(),
                        activation(field.getAnnotation(LoopSchedule.class)),
                        LoopDeclarationKind.FIELD,
                        definition.controller(),
                        field.getType()))
                    .orElseGet(() -> new MechanismLoopDefinition(
                        publicName,
                        controlLoop.output(),
                        activation(field.getAnnotation(LoopSchedule.class)),
                        LoopDeclarationKind.FIELD,
                        MechanismCustomControllerDefinition.EMPTY,
                        field.getType()));
                loops.add(declared);
            }
        }

        for (Method method : type.getDeclaredMethods()) {
            ControlLoop controlLoop = method.getAnnotation(ControlLoop.class);
            if (controlLoop != null) {
                loops.add(new MechanismLoopDefinition(
                    DeclarationNameResolver.publicName(method, controlLoop),
                    controlLoop.output(),
                    activation(method.getAnnotation(LoopSchedule.class)),
                    LoopDeclarationKind.METHOD,
                    MechanismCustomControllerDefinition.of(
                        AnnotatedMechanismCallbackFactory.loopCallback(method, declaration)),
                    method.getReturnType()));
            }
            collectAutomation(automation, method, declaration);
        }

        Optional<Object> initialStateObject = initialStateObject(declaration);
        Optional<Class<?>> stateType = initialStateObject.map(Object::getClass)
                .or(() -> AnnotatedMechanismCallbackFactory.stateType(type));
        Optional<String> initialStateName = initialState == null || initialState.value().isBlank()
                ? initialStateObject.map(StateNames::name)
                : Optional.of(initialState.value().trim());

        return new MechanismDefinition(
            name,
            identity.disabled(),
            stateType,
            initialStateName,
            initialStateObject.or(() -> resolveInitialState(stateType, initialStateName)),
            0.0,
            identity,
            motors,
            encoders,
            inputs,
            loops,
            automation);
    }

    private static Optional<Object> initialStateObject(Optional<Object> declaration) {
        if (declaration.isEmpty()) {
            return Optional.empty();
        }
        Object value = declaration.orElseThrow();
        if (value instanceof StatefulMechanismConfig stateful) {
            return Optional.ofNullable(stateful.initialState());
        }
        if (value instanceof TypedStatefulMechanismConfig<?> stateful) {
            return Optional.ofNullable(stateful.initialState());
        }
        return Optional.empty();
    }

    @SuppressWarnings({"rawtypes", "unchecked"})
    private static Optional<Object> resolveInitialState(Optional<Class<?>> stateType, Optional<String> initialStateName) {
        if (stateType.isEmpty() || initialStateName.isEmpty()) {
            return Optional.empty();
        }
        Class<?> type = stateType.orElseThrow();
        if (!Enum.class.isAssignableFrom(type)) {
            return Optional.empty();
        }
        return Optional.of(Enum.valueOf((Class<? extends Enum>) type.asSubclass(Enum.class), initialStateName.orElseThrow()));
    }

    private static String mechanismName(Class<?> type) {
        Mechanism mechanism = type.getAnnotation(Mechanism.class);
        if (mechanism == null || mechanism.value().isBlank()) {
            return type.getSimpleName();
        }
        return mechanism.value().trim();
    }

    private static MechanismIdentity identity(Class<?> type) {
        PositionDomain positionDomain = type.getAnnotation(PositionDomain.class);
        ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange travelRange =
            type.getAnnotation(ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange.class);

        return new MechanismIdentity(
            positionDomain == null
                ? Optional.empty()
                : Optional.of(new PositionDomainSpec(positionDomain.value(), positionDomain.units())),
            travelRange == null
                ? Optional.empty()
                : Optional.of(new TravelRange(travelRange.min(), travelRange.max())),
            type.isAnnotationPresent(ContinuousRotation.class),
            false);
    }

    private static LoopActivation activation(LoopSchedule schedule) {
        if (schedule == null) {
            return LoopActivation.enabled();
        }
        return new LoopActivation(schedule.mode(), List.of(schedule.states()));
    }

    private static Optional<String> nonBlank(String value) {
        String trimmed = value == null ? "" : value.trim();
        return trimmed.isEmpty() ? Optional.empty() : Optional.of(trimmed);
    }

    private static OptionalInt hardwareId(int id) {
        return id == Integer.MIN_VALUE ? OptionalInt.empty() : OptionalInt.of(id);
    }

    private static OptionalDouble finite(double value) {
        return Double.isFinite(value) ? OptionalDouble.of(value) : OptionalDouble.empty();
    }

    private static Optional<Object> fieldValue(Field field, Optional<Object> declaration) {
        return AnnotatedMechanismCallbackFactory.fieldValue(field, declaration);
    }

    private static Optional<MechanismLoopDefinition> fieldLoopDefinition(
            Field field,
            Optional<Object> declaration,
            String inferredName) {
        return fieldValue(field, declaration)
            .flatMap(value -> {
                if (value instanceof MechanismPid pid) {
                    pid.named(inferredName);
                    return Optional.of(pid.definition());
                }
                if (value instanceof MechanismFeedforward feedforward) {
                    feedforward.named(inferredName);
                    return Optional.of(feedforward.definition());
                }
                if (value instanceof MechanismControlLoop loop) {
                    loop.named(inferredName);
                    return Optional.of(loop.definition());
                }
                return Optional.empty();
            });
    }

    private static void collectAutomation(
            List<MechanismAutomationDefinition> automation,
            Method method,
            Optional<Object> declaration) {
        OnStateEnter enter = method.getAnnotation(OnStateEnter.class);
        if (enter != null) {
            automation.add(new MechanismAutomationDefinition(
                    MechanismAutomationPhase.STATE_ENTER,
                    List.of(enter.value()),
                    AnnotatedMechanismCallbackFactory.automationCallback(method, declaration)));
        }
        OnStatePeriodic periodic = method.getAnnotation(OnStatePeriodic.class);
        if (periodic != null) {
            automation.add(new MechanismAutomationDefinition(
                    MechanismAutomationPhase.STATE_PERIODIC,
                    List.of(periodic.value()),
                    AnnotatedMechanismCallbackFactory.automationCallback(method, declaration)));
        }
        OnStateExit exit = method.getAnnotation(OnStateExit.class);
        if (exit != null) {
            automation.add(new MechanismAutomationDefinition(
                    MechanismAutomationPhase.STATE_EXIT,
                    List.of(exit.value()),
                    AnnotatedMechanismCallbackFactory.automationCallback(method, declaration)));
        }
    }
}
