package ca.frc6390.athena.auto;

/** Creates autonomous choosers whose entries are ordinary Athena Actions. */
public final class Autos {
    private Autos() { }

    /** Creates the standard dashboard auto chooser. */
    public static AutoChooser chooser() {
        return chooser("Auto Chooser");
    }

    public static AutoChooser chooser(String dashboardName) {
        return new AutoChooser(dashboardName);
    }
}
