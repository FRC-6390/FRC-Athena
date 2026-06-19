package ca.frc6390.athena.api.mechanism.introspection;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopSchedule;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.input.BooleanInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DigitalInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DoubleInput;
import ca.frc6390.athena.mechanisms.OutputType;

class DeclarationNameResolverTest {
    @Test
    void defaultsFieldBackedNamesToFieldNames() throws Exception {
        Field encoder = SampleDeclarations.class.getDeclaredField("turretCrt");
        Field digital = SampleDeclarations.class.getDeclaredField("home");
        Field boolInput = SampleDeclarations.class.getDeclaredField("allowCounterRotation");
        Field doubleInput = SampleDeclarations.class.getDeclaredField("fieldHeadingDeg");
        Field pidLoop = SampleDeclarations.class.getDeclaredField("mainPid");

        assertEquals("turretCrt", DeclarationNameResolver.publicName(encoder, encoder.getAnnotation(Encoder.class)));
        assertEquals("home", DeclarationNameResolver.publicName(digital, digital.getAnnotation(DigitalInput.class)));
        assertEquals("allowCounterRotation",
            DeclarationNameResolver.publicName(boolInput, boolInput.getAnnotation(BooleanInput.class)));
        assertEquals("fieldHeadingDeg",
            DeclarationNameResolver.publicName(doubleInput, doubleInput.getAnnotation(DoubleInput.class)));
        assertEquals("mainPid", DeclarationNameResolver.publicName(pidLoop, pidLoop.getAnnotation(ControlLoop.class)));
    }

    @Test
    void preservesExplicitAliasesWhenPresent() throws Exception {
        Field aliasedEncoder = SampleDeclarations.class.getDeclaredField("derivedEncoder");
        Method aliasedLoop = SampleDeclarations.class.getDeclaredMethod("hubTargetingLoop");

        assertEquals("turretCrtAlias",
            DeclarationNameResolver.publicName(aliasedEncoder, aliasedEncoder.getAnnotation(Encoder.class)));
        assertEquals("hubAim",
            DeclarationNameResolver.publicName(aliasedLoop, aliasedLoop.getAnnotation(ControlLoop.class)));
    }

    @Test
    void defaultsMethodBackedLoopNamesToMethodNames() throws Exception {
        Method method = SampleDeclarations.class.getDeclaredMethod("teleHoldLoop");
        assertEquals("teleHoldLoop",
            DeclarationNameResolver.publicName(method, method.getAnnotation(ControlLoop.class)));
    }

    @SuppressWarnings("unused")
    private static final class SampleDeclarations {
        @Encoder
        Object turretCrt;

        @Encoder("turretCrtAlias")
        Object derivedEncoder;

        @DigitalInput(port = -6)
        Object home;

        @BooleanInput
        Object allowCounterRotation;

        @DoubleInput
        Object fieldHeadingDeg;

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        Object mainPid;

        @ControlLoop("hubAim")
        @LoopSchedule(mode = LoopMode.ENABLED, states = {"Aim"})
        double hubTargetingLoop() {
            return 0.0;
        }

        @ControlLoop
        @LoopSchedule(mode = LoopMode.TELE)
        double teleHoldLoop() {
            return 0.0;
        }
    }
}
