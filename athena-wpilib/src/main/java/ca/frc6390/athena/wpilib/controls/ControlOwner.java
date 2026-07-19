package ca.frc6390.athena.wpilib.controls;

/** Internal registration contract shared by mapped and raw-ID controllers. */
abstract class ControlOwner {
    abstract void register(ControlSignal signal);

    abstract void register(AxisSignal axis);

    abstract boolean connected();
}
