package frc.robot;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.vendor.choreo.ChoreoPathProvider;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.auto.AutoContext;
import frc.robot.auto.ChoreoMarkersAuto;
import frc.robot.auto.ChoreoMultiPathSplitAuto;
import frc.robot.auto.ConditionalChoreoAuto;
import frc.robot.auto.ExampleDrive;

public final class Robot extends AthenaRobot {
    public final ExampleDrive drive = new ExampleDrive();
    public final AutoContext autoContext = new AutoContext(drive);
    public final ChoreoPathProvider paths = autoContext.choreo;
    public final AutoChooser autos = Autos.chooser()
            .defaultAuto(ChoreoMarkersAuto.create(autoContext))
            .auto(ChoreoMultiPathSplitAuto.create(autoContext))
            .auto(ConditionalChoreoAuto.create(autoContext));
}
