package ca.frc6390.athena.examples;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import ca.frc6390.athena.hardware.config.MotorConfig;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.vendor.ctre.CtreMotorOptions;
import ca.frc6390.athena.vendor.rev.RevMotorOptions;

/**
 * Examples for typed vendor-specific motor options.
 *
 * <p>These declarations require only Athena vendor adapter artifacts at compile
 * time. The generic hardware module still does not import CTRE or REV classes.</p>
 */
public final class VendorOptionsExample {
    private VendorOptionsExample() {
    }

    /**
     * Creates a CTRE Kraken motor spec with CTRE-specific current limits.
     *
     * @return CTRE motor spec
     */
    public static MotorSpec ctreShooterLeader() {
        return MotorConfig.create()
                .hardware(AthenaMotor.KRAKEN_X60, 12)
                .canbus("canivore")
                .coast()
                .currentLimit(60)
                .integratedEncoder()
                .vendor(CtreMotorOptions.class, ctre -> ctre
                        .supplyCurrentLimit(50)
                        .statorCurrentLimit(80)
                        .torqueCurrentLimit(120))
                .toSpec("shooter", "leader");
    }

    /**
     * Creates a REV Spark Flex motor spec with REV-specific ramping options.
     *
     * @return REV motor spec
     */
    public static MotorSpec revArmPivot() {
        return MotorConfig.create()
                .hardware(AthenaMotor.SPARK_FLEX_BRUSHLESS, 21)
                .brake()
                .currentLimit(50)
                .integratedEncoder()
                .vendor(RevMotorOptions.class, rev -> rev
                        .smartCurrentLimit(50)
                        .openLoopRampSeconds(0.2)
                        .closedLoopRampSeconds(0.1))
                .toSpec("arm", "pivot");
    }

    /**
     * Creates a CTRE CANcoder spec for a swerve steering encoder.
     *
     * @return CTRE encoder spec
     */
    public static EncoderSpec ctreSteerEncoder() {
        return EncoderConfig.create()
                .hardware(AthenaEncoder.CANCODER, 22)
                .canbus("canivore")
                .absolutePosition()
                .offset(0.125)
                .toSpec("swerve.frontLeft", "steer");
    }
}
