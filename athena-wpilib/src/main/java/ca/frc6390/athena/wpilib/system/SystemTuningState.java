package ca.frc6390.athena.wpilib.system;

record SystemTuningState(
        String overcommitMemory,
        String swappiness,
        boolean webServerRunning,
        boolean athenaZram,
        boolean athenaSwapFile) {

    SystemTuningState withZram(boolean value) {
        return new SystemTuningState(overcommitMemory, swappiness, webServerRunning, value, athenaSwapFile);
    }

    SystemTuningState withSwapFile(boolean value) {
        return new SystemTuningState(overcommitMemory, swappiness, webServerRunning, athenaZram, value);
    }
}
