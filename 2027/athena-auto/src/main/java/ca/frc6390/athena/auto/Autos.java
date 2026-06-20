package ca.frc6390.athena.auto;

/**
 * Entry points for autonomous declarations.
 */
public final class Autos {
    private Autos() {
    }

    /**
     * Creates an auto chooser declaration.
     *
     * @return chooser config
     */
    public static AutoChooserConfig chooser() {
        return new AutoChooserConfig();
    }
}
