package frc.robot.utils;

public class Log {
    public static void info(String message) {
        // blue bold color
        System.out.println("\033[0;34;1mINFO " + message + "\033[0m");
    }

    public static void error(String message) {
        // red bold color
        System.out.println("\033[0;31;1mERROR " + message + "\033[0m");
    }

    public static void warning(String message) {
        // yellow bold color
        System.out.println("\033[0;33;1mWARNING " + message + "\033[0m");
    }

    public static void debug(String message) {
        // green bold color
        System.out.println("\033[0;32;1mDEBUG " + message + "\033[0m");
    }
}
