package ca.frc6390.athena.wpilib.system;

interface SystemAccess {
    record Memory(long total, long available, long rss, long swapTotal, long swapUsed) { }

    boolean realRobot();
    boolean linux();
    String target();
    Memory memory();
    String swapKind();
    String readSysctl(String key);
    boolean niWebServerRunning();
    boolean zramSupported();
    boolean hasActiveZram();
    boolean hasActiveSwapFile();
    long usableSwapDiskBytes();
    Result stopNiWebServer();
    Result startNiWebServer();
    Result setSysctl(String key, int value);
    Result enableZram(long bytes);
    Result disableZram();
    Result enableSwapFile(long bytes, long minimumFreeDiskBytes);
    Result disableSwapFile();

    record Result(boolean success, String detail) {
        static Result success(String detail) { return new Result(true, detail); }
        static Result failure(String detail) { return new Result(false, detail); }
    }
}
