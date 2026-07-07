package ca.frc6390.athena.drivetrain;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.drivetrain.spec.WheelBase;
import ca.frc6390.athena.drivetrain.swerve.SwerveDrivebaseDefinition;
import ca.frc6390.athena.drivetrain.swerve.SwerveDrivebaseIntrospector;
import ca.frc6390.athena.drivetrain.swerve.SwerveDrivebase;
import ca.frc6390.athena.drivetrain.swerve.ModuleLocationRef;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleOrder;
import ca.frc6390.athena.drivetrain.swerve.SwerveModules;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.ImuRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.mechanism.core.ControlRef;
import ca.frc6390.athena.mechanism.core.Control;
import ca.frc6390.athena.mechanism.core.MechanismDefinition;
import ca.frc6390.athena.mechanism.core.MechanismIntrospector;
import org.junit.jupiter.api.Test;

class SwerveDrivebaseIntrospectionTest {
    @Test
    void swerveModulesAreDiscoveredAsChildMechanismsWithoutUserStates() {
        TestDrivebase drivebase = new TestDrivebase();

        MechanismDefinition definition = MechanismIntrospector.inspect(drivebase);

        assertTrue(definition.children().containsKey("frontLeft"));
        assertTrue(definition.children().containsKey("frontRight"));
        assertDoesNotThrow(drivebase.frontLeft::initialState);
        assertDoesNotThrow(drivebase.frontRight::initialState);
    }

    @Test
    void drivebaseIntrospectorSortsAnnotatedModulesAndDerivesFourModuleLocations() {
        RectangularDrivebase drivebase = new RectangularDrivebase();

        SwerveDrivebaseDefinition definition = SwerveDrivebaseIntrospector.inspect(drivebase);

        assertEquals(4, definition.modules().size());
        assertEquals("FrontLeft", definition.modules().get(0).name());
        assertEquals("FrontRight", definition.modules().get(1).name());
        assertEquals("BackLeft", definition.modules().get(2).name());
        assertEquals("BackRight", definition.modules().get(3).name());
        assertEquals(0.3, definition.modules().get(0).location().xMeters(), 1.0e-9);
        assertEquals(0.25, definition.modules().get(0).location().yMeters(), 1.0e-9);
        assertEquals(-0.3, definition.modules().get(3).location().xMeters(), 1.0e-9);
        assertEquals(-0.25, definition.modules().get(3).location().yMeters(), 1.0e-9);
        assertEquals(AthenaImu.PIGEON_2, definition.imu().kind());
        assertEquals(0, definition.imu().id());
        assertTrue(definition.modules().get(0).controls().containsKey("driveControl"));
        assertTrue(definition.modules().get(0).controls().containsKey("steerControl"));
    }

    @Test
    void drivebaseIntrospectorUsesClassOrderForFieldBackedModules() {
        FieldBackedDrivebase drivebase = new FieldBackedDrivebase();

        SwerveDrivebaseDefinition definition = SwerveDrivebaseIntrospector.inspect(drivebase);

        assertEquals("frontLeft", definition.modules().get(0).name());
        assertEquals("frontRight", definition.modules().get(1).name());
        assertEquals("backLeft", definition.modules().get(2).name());
        assertEquals("backRight", definition.modules().get(3).name());
    }

    @Test
    void drivebaseIntrospectorAllowsThreePlusModulesWithExplicitLocations() {
        TriModuleDrivebase drivebase = new TriModuleDrivebase();

        SwerveDrivebaseDefinition definition = SwerveDrivebaseIntrospector.inspect(drivebase);

        assertEquals(3, definition.modules().size());
        assertEquals("Front", definition.modules().get(0).name());
        assertEquals(0.2, definition.modules().get(0).location().xMeters(), 1.0e-9);
        assertTrue(definition.modules().get(0).explicitLocation());
    }

    @Test
    void drivebaseIntrospectorRejectsModulesWithoutLocationOrDerivableGeometry() {
        assertThrows(
                IllegalStateException.class,
                () -> SwerveDrivebaseIntrospector.inspect(new MissingLocationDrivebase()));
    }

    private static final class TestDrivebase implements SwerveDrivebase {
        private final FrontLeft frontLeft = new FrontLeft();
        private final FrontRight frontRight = new FrontRight();
    }

    private static final class FrontLeft extends SwerveModules.SDS.MK5N.R3 {
        private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 1);
        private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 2);
        private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 3).absolutePosition();
    }

    private static final class FrontRight extends SwerveModules.SDS.MK5N.R3 {
        private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 4);
        private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 5);
        private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 6).absolutePosition();
    }

    private static final class RectangularDrivebase implements SwerveDrivebase {
        private final TrackWidth trackWidth = TrackWidth.meters(0.5);
        private final WheelBase wheelBase = WheelBase.meters(0.6);
        private final ImuRef imu = ImuRef.of(AthenaImu.PIGEON_2, 0);

        @SwerveModuleOrder(2)
        private static final class BackLeft extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 7);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 8);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 9).absolutePosition();
        }

        @SwerveModuleOrder(0)
        private static final class FrontLeft extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 10);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 11);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 12).absolutePosition();
            private final ControlRef driveControl = Control.velocity(drive).feedback(drive.encoder());
            private final ControlRef steerControl = Control.position(steer).feedback(angle).pid(0.3, 0.0, 0.0);
        }

        @SwerveModuleOrder(3)
        private static final class BackRight extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 13);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 14);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 15).absolutePosition();
        }

        @SwerveModuleOrder(1)
        private static final class FrontRight extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 16);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 17);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 18).absolutePosition();
        }
    }

    private static final class TriModuleDrivebase implements SwerveDrivebase {
        @SwerveModuleOrder(0)
        private static final class Front extends SwerveModules.Custom {
            private final ModuleLocationRef location = ModuleLocationRef.meters(0.2, 0.0);
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 19);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 20);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 21).absolutePosition();
        }

        @SwerveModuleOrder(1)
        private static final class LeftRear extends SwerveModules.Custom {
            private final ModuleLocationRef location = ModuleLocationRef.meters(-0.2, 0.15);
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 22);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 23);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 24).absolutePosition();
        }

        @SwerveModuleOrder(2)
        private static final class RightRear extends SwerveModules.Custom {
            private final ModuleLocationRef location = ModuleLocationRef.meters(-0.2, -0.15);
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 25);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 26);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 27).absolutePosition();
        }
    }

    private static final class FieldBackedDrivebase implements SwerveDrivebase {
        private final TrackWidth trackWidth = TrackWidth.meters(0.5);
        private final WheelBase wheelBase = WheelBase.meters(0.6);
        private final FieldBackRight backRight = new FieldBackRight();
        private final FieldFrontLeft frontLeft = new FieldFrontLeft();
        private final FieldBackLeft backLeft = new FieldBackLeft();
        private final FieldFrontRight frontRight = new FieldFrontRight();

        @SwerveModuleOrder(0)
        private static final class FieldFrontLeft extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 28);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 29);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 30).absolutePosition();
        }

        @SwerveModuleOrder(1)
        private static final class FieldFrontRight extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 31);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 32);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 33).absolutePosition();
        }

        @SwerveModuleOrder(2)
        private static final class FieldBackLeft extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 34);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 35);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 36).absolutePosition();
        }

        @SwerveModuleOrder(3)
        private static final class FieldBackRight extends SwerveModules.SDS.MK5N.R3 {
            private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 37);
            private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 38);
            private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 39).absolutePosition();
        }
    }

    private static final class MissingLocationDrivebase implements SwerveDrivebase {
        private final SingleModule first = new SingleModule();
        private final SingleModule second = new SingleModule();
        private final SingleModule third = new SingleModule();
    }

    private static final class SingleModule extends SwerveModules.Custom {
        private final MotorRef drive = MotorRef.of(AthenaMotor.SIM, 22);
        private final MotorRef steer = MotorRef.of(AthenaMotor.SIM, 23);
        private final EncoderRef angle = EncoderRef.of(AthenaEncoder.SIM, 24).absolutePosition();
    }
}
