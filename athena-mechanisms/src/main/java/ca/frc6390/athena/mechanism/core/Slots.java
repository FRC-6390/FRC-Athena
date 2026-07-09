package ca.frc6390.athena.mechanism.core;

/**
 * Factories for mechanism-template slots.
 */
public final class Slots {
    private Slots() {
    }

    public static <O, T> Slot<O, T> of(O owner, String name, Runnable onFill) {
        return new Slot<>(owner, name, onFill);
    }

    public static <O> MotorSlot<O> motor(O owner, String name, Runnable onFill) {
        return new MotorSlot<>(owner, name, onFill);
    }

    public static <O> EncoderSlot<O> encoder(O owner, String name, Runnable onFill) {
        return new EncoderSlot<>(owner, name, onFill);
    }

    public static <O> ImuSlot<O> imu(O owner, String name, Runnable onFill) {
        return new ImuSlot<>(owner, name, onFill);
    }

    public static <O> DigitalInputSlot<O> digitalInput(O owner, String name, Runnable onFill) {
        return new DigitalInputSlot<>(owner, name, onFill);
    }
}
