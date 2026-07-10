package frc.robot;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;
import frc.robot.auto.AutoExamples;

public final class Robot extends AthenaRobot {
    public final AutoExamples autos = new AutoExamples();

    @SuppressWarnings("unused")
    public final HookBinding selectDefault = Events.robotInit()
            .onStart(() -> autos.runtime.select("Practice timed path"));

    @Override
    protected void configure() {
        athena().auto(autos.runtime, autos.markers);
        autos.loadProviderExamples();
    }
}
