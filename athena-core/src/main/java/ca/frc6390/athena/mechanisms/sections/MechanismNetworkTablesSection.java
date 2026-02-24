package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotNetworkTables;

public final class MechanismNetworkTablesSection {
    private final Mechanism owner;

    MechanismNetworkTablesSection(Mechanism owner) {
        this.owner = owner;
    }

    public MechanismNetworkTablesSection periodSeconds(double periodSeconds) {
        owner.networkTablesPeriodSecondsInternal(periodSeconds);
        return this;
    }

    public double periodSeconds() {
        return owner.networkTablesPeriodSecondsInternal();
    }

    public MechanismNetworkTablesSection ownerPath(String ownerPath) {
        owner.networkTablesOwnerPathInternal(ownerPath);
        return this;
    }

    public String ownerPath() {
        return owner.networkTablesOwnerPathInternal();
    }

    public MechanismNetworkTablesSection publishHint(String ownerHint) {
        owner.networkTablesPublishHintInternal(ownerHint);
        return this;
    }

    public RobotNetworkTables.MechanismToggles toggles() {
        return owner.networkTablesTogglesInternal();
    }

    public String typeName() {
        return owner.networkTablesTypeNameInternal();
    }

    public RobotNetworkTables.Node resolveNode(RobotNetworkTables.Node mechanismsRoot) {
        return owner.resolveNetworkTablesNodeInternal(mechanismsRoot);
    }
}
