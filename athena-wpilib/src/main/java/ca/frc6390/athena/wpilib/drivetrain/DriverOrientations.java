package ca.frc6390.athena.wpilib.drivetrain;

import ca.frc6390.athena.drivetrain.DriverOrientation;
import ca.frc6390.athena.localization.pipeline.Localization;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Builds a {@link DriverOrientation} wired to the Driver Station and a {@link Localization}.
 *
 * <p>The alliance comes from {@link DriverStation#getAlliance()} (unknown until the Driver Station reports it), the
 * pose from {@link Localization#pose()}, seeding from {@link Localization#seeded()}, and the pose reset from
 * {@link Localization#reset(ca.frc6390.athena.runtime.filter.PoseSnapshot)}.</p>
 *
 * <p>The localization may be constructed after the drivetrain that owns the orientation. Pass a supplier that reads
 * the field, and it is resolved on every call, not at construction.</p>
 */
public final class DriverOrientations {
    private DriverOrientations() {
    }

    /**
     * Returns the orientation for an already-constructed localization.
     *
     * @param localization localization that owns the field pose
     * @return driver orientation
     */
    public static DriverOrientation forDriverStation(Localization localization) {
        Objects.requireNonNull(localization, "localization");
        return forDriverStation(() -> localization);
    }

    /**
     * Returns the orientation for a localization that is bound after the owning mechanism is constructed.
     *
     * @param localization supplier of the localization, resolved on each use
     * @return driver orientation
     */
    public static DriverOrientation forDriverStation(Supplier<Localization> localization) {
        return forAlliance(DriverOrientations::driverStationAlliance, localization);
    }

    /** Returns the Driver Station alliance mapped to the driver orientation alliance. Empty while unknown. */
    public static Optional<DriverOrientation.Alliance> driverStationAlliance() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red
                ? DriverOrientation.Alliance.RED
                : DriverOrientation.Alliance.BLUE);
    }

    /** Package-private so tests can supply the alliance without a Driver Station. */
    static DriverOrientation forAlliance(
            Supplier<Optional<DriverOrientation.Alliance>> alliance,
            Supplier<Localization> localization) {
        Objects.requireNonNull(alliance, "alliance");
        Supplier<Localization> bound = Objects.requireNonNull(localization, "localization");
        return DriverOrientation.of(
                () -> resolve(bound).pose(),
                alliance,
                () -> resolve(bound).seeded(),
                pose -> resolve(bound).reset(pose));
    }

    private static Localization resolve(Supplier<Localization> localization) {
        Localization value = localization.get();
        if (value == null) {
            throw new IllegalStateException("The driver orientation's localization is not bound yet.");
        }
        return value;
    }
}
