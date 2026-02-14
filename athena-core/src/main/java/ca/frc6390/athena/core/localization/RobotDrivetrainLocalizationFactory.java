package ca.frc6390.athena.core.localization;

public interface RobotDrivetrainLocalizationFactory {
    RobotLocalization<?> createLocalization(RobotLocalizationConfig config);
}
