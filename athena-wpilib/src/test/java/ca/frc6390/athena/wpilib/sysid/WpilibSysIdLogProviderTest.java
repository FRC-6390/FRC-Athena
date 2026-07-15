package ca.frc6390.athena.wpilib.sysid;

import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.sysid.SysIdLogProvider;
import java.util.ServiceLoader;
import org.junit.jupiter.api.Test;

class WpilibSysIdLogProviderTest {
    @Test
    void providerIsDiscoverableFromTheWpilibModule() {
        boolean found = ServiceLoader.load(SysIdLogProvider.class).stream()
                .anyMatch(provider -> provider.type() == WpilibSysIdLogProvider.class);

        assertTrue(found);
    }
}
