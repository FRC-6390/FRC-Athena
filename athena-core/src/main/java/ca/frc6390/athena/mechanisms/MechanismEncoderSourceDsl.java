package ca.frc6390.athena.mechanisms;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;

/**
 * Top-level view of the encoder-source fluent API so IDE completion does not
 * depend on inferring a nested member type inside another lambda.
 */
public interface MechanismEncoderSourceDsl {

    MechanismEncoderSourceDsl config(EncoderConfig config);

    MechanismEncoderSourceDsl encoder(AthenaEncoder type, int id);

    MechanismEncoderSourceDsl encoder(EncoderType type, int id);

    MechanismEncoderSourceDsl virtual(DoubleSupplier positionSupplier);

    MechanismEncoderSourceDsl virtual(MechanismConfig.VirtualSourceSection section);

    MechanismEncoderSourceDsl crt(MechanismConfig.CrtSourceSection section);

    MechanismEncoderSourceDsl filter(MechanismConfig.FilterSourceSection section);

    MechanismEncoderSourceDsl differentiate(MechanismConfig.DifferentiateSourceSection section);

    MechanismEncoderSourceDsl average(MechanismConfig.AverageSourceSection section);

    MechanismEncoderSourceDsl difference(MechanismConfig.DifferenceSourceSection section);

    MechanismEncoderSourceDsl calibrationMap(MechanismConfig.CalibrationMapSourceSection section);

    MechanismEncoderSourceDsl canbus(String canbus);

    MechanismEncoderSourceDsl gearRatio(double gearRatio);

    MechanismEncoderSourceDsl conversion(double conversion);

    MechanismEncoderSourceDsl offset(double offset);

    MechanismEncoderSourceDsl unit(MechanismEncoderUnit unit);

    MechanismEncoderSourceDsl wrapsEvery(double wrapsEvery);
}
