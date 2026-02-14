package ca.frc6390.athena.hardware.config;

import java.util.function.Consumer;
import java.util.function.IntConsumer;

/**
 * Shared fluent hardware identity section for device configs.
 */
public abstract class DeviceIdentitySection<S, TType> {
    private final Consumer<TType> typeApplier;
    private final IntConsumer idApplier;
    private final Consumer<String> canbusApplier;
    private final Consumer<Boolean> invertedApplier;

    protected DeviceIdentitySection(
            Consumer<TType> typeApplier,
            IntConsumer idApplier,
            Consumer<String> canbusApplier,
            Consumer<Boolean> invertedApplier) {
        this.typeApplier = typeApplier;
        this.idApplier = idApplier;
        this.canbusApplier = canbusApplier;
        this.invertedApplier = invertedApplier;
    }

    protected abstract S self();

    public S type(TType type) {
        typeApplier.accept(type);
        return self();
    }

    public S id(int id) {
        idApplier.accept(id);
        return self();
    }

    public S canbus(String canbus) {
        canbusApplier.accept(canbus);
        return self();
    }

    public S inverted(boolean inverted) {
        invertedApplier.accept(inverted);
        return self();
    }
}
