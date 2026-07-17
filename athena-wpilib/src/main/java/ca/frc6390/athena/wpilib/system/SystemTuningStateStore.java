package ca.frc6390.athena.wpilib.system;

import java.nio.file.Path;
import java.util.Optional;

interface SystemTuningStateStore {
    Optional<SystemTuningState> load();
    SystemAccess.Result save(SystemTuningState state);
    SystemAccess.Result delete();

    static SystemTuningStateStore rio() {
        return new PropertiesStateStore(Path.of("/home/lvuser/athena/system-state.properties"));
    }
}
