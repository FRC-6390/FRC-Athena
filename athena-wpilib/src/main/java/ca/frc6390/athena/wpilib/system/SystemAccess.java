package ca.frc6390.athena.wpilib.system;

interface SystemAccess {
    record Memory(long total, long available, long rss, long swapTotal, long swapUsed) { }

    boolean realRobot();

    boolean linux();

    String target();

    Memory memory();

    boolean hasActiveZram();

    Result stopNiWebServer();

    Result setSysctl(String key, int value);

    Result enableSwapFile(long bytes);

    record Result(boolean success, String detail) {
        static Result success(String detail) {
            return new Result(true, detail);
        }

        static Result failure(String detail) {
            return new Result(false, detail);
        }
    }
}
